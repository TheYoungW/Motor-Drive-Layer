#include "articore/dds/lease.hpp"

#include <utility>

namespace articore::dds {

LeaseManager::LeaseManager(LostCallback on_lost,
                           std::chrono::milliseconds timeout)
    : on_lost_(std::move(on_lost)), timeout_(timeout) {}

Result<LeaseSnapshot> LeaseManager::acquire(const std::string& client_id,
                                            Clock::time_point now) {
  if (client_id.empty() || client_id.size() > 63) {
    return Status::failure(RuntimeErrorCode::InvalidArgument,
                           "client_id must contain 1..63 bytes");
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (active_ && now < active_->expires_at &&
      active_->client_id != client_id) {
    return Status::failure(RuntimeErrorCode::Busy,
                           "another client owns the robot lease");
  }
  if (!active_ || active_->client_id != client_id ||
      now >= active_->expires_at) {
    active_ = LeaseSnapshot{client_id, next_lease_id_++, now + timeout_};
    last_sequences_.erase(client_id);
  } else {
    active_->expires_at = now + timeout_;
  }
  return *active_;
}

ProtocolError LeaseManager::authorize(const std::string& client_id,
                                      std::uint64_t lease_id,
                                      std::uint64_t sequence, bool refresh,
                                      Clock::time_point now) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_ || now >= active_->expires_at ||
      active_->client_id != client_id || active_->lease_id != lease_id) {
    return ProtocolError::NoLease;
  }
  const auto found = last_sequences_.find(client_id);
  if (found != last_sequences_.end() && sequence <= found->second) {
    return ProtocolError::StaleSequence;
  }
  last_sequences_[client_id] = sequence;
  if (refresh) active_->expires_at = now + timeout_;
  return ProtocolError::Ok;
}

ProtocolError LeaseManager::heartbeat(const std::string& client_id,
                                      std::uint64_t lease_id,
                                      std::uint64_t sequence,
                                      Clock::time_point now) {
  return authorize(client_id, lease_id, sequence, true, now);
}

ProtocolError LeaseManager::release(const std::string& client_id,
                                    std::uint64_t lease_id,
                                    const std::string& reason) {
  bool released = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_ || active_->client_id != client_id ||
        active_->lease_id != lease_id) {
      return ProtocolError::NoLease;
    }
    active_.reset();
    released = true;
  }
  if (released) notify_lost(reason);
  return ProtocolError::Ok;
}

bool LeaseManager::expire_if_needed(Clock::time_point now) {
  bool expired = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_ && now >= active_->expires_at) {
      active_.reset();
      expired = true;
    }
  }
  if (expired) notify_lost("control lease expired");
  return expired;
}

void LeaseManager::revoke(const std::string& reason) {
  bool had_lease = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    had_lease = active_.has_value();
    active_.reset();
  }
  if (had_lease) notify_lost(reason);
}

std::optional<LeaseSnapshot> LeaseManager::snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return active_;
}

void LeaseManager::notify_lost(const std::string& reason) {
  if (on_lost_) on_lost_(reason);
}

}  // namespace articore::dds
