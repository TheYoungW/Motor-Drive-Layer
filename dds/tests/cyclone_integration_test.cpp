#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>

#include "articore/dds/service.hpp"
#include "articore_protocol.h"
#include "dds/dds.h"

namespace {
using namespace std::chrono_literals;

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

void check(dds_return_t result, const char* message) {
  if (result < 0) {
    throw std::runtime_error(std::string(message) + ": " +
                             dds_strretcode(-result));
  }
}

articore_wire_ControlReply wait_reply(dds_entity_t reader,
                                      std::uint64_t request_id) {
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (std::chrono::steady_clock::now() < deadline) {
    articore_wire_ControlReply reply{};
    void* samples[1]{&reply};
    dds_sample_info_t infos[1]{};
    const auto count = dds_take(reader, samples, infos, 1, 1);
    if (count > 0 && infos[0].valid_data && reply.request_id == request_id) {
      return reply;
    }
    std::this_thread::sleep_for(5ms);
  }
  throw std::runtime_error("timed out waiting for correlated control reply");
}

void initialize(articore_wire_ControlRequest& request, const char* client,
                std::uint64_t sequence, std::uint64_t request_id) {
  request.protocol_major = 1;
  request.protocol_minor = 0;
  std::snprintf(request.robot_id, sizeof(request.robot_id), "%s", "loopback-robot");
  std::snprintf(request.client_id, sizeof(request.client_id), "%s", client);
  request.sequence_id = sequence;
  request.request_id = request_id;
}
}  // namespace

int main() {
  articore::dds::ServiceConfig config;
  config.robot_id = "loopback-robot";
  config.domain_id = 73;
  config.network_interfaces = {"lo"};
  config.runtime.left_can_interface = "missing-can-left";
  config.runtime.right_can_interface = "missing-can-right";
  articore::dds::CycloneService service(config);
  std::atomic<bool> stop{false};
  articore::Status service_status;
  std::thread service_thread([&] { service_status = service.run(stop); });

  const auto participant = dds_create_participant(config.domain_id, nullptr, nullptr);
  check(participant, "create client participant");
  const auto request_topic = dds_create_topic(
      participant, &articore_wire_ControlRequest_desc,
      "articore.robot.control.request", nullptr, nullptr);
  check(request_topic, "create request topic");
  const auto reply_topic = dds_create_topic(
      participant, &articore_wire_ControlReply_desc,
      "articore.robot.control.reply", nullptr, nullptr);
  check(reply_topic, "create reply topic");
  const auto stream_topic = dds_create_topic(
      participant, &articore_wire_StreamCommand_desc,
      "articore.robot.stream.command", nullptr, nullptr);
  check(stream_topic, "create stream topic");
  auto* qos = dds_create_qos();
  dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_MSECS(20));
  dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 32);
  const auto request_writer = dds_create_writer(
      participant, request_topic, qos, nullptr);
  check(request_writer, "create request writer");
  const auto reply_reader = dds_create_reader(
      participant, reply_topic, qos, nullptr);
  check(reply_reader, "create reply reader");
  dds_delete_qos(qos);
  auto* stream_qos = dds_create_qos();
  dds_qset_reliability(
      stream_qos, DDS_RELIABILITY_BEST_EFFORT, DDS_MSECS(0));
  dds_qset_history(stream_qos, DDS_HISTORY_KEEP_LAST, 1);
  dds_qset_lifespan(stream_qos, DDS_MSECS(20));
  const auto stream_writer = dds_create_writer(
      participant, stream_topic, stream_qos, nullptr);
  check(stream_writer, "create stream writer");
  dds_delete_qos(stream_qos);
  std::this_thread::sleep_for(250ms);

  articore_wire_ControlRequest first{};
  initialize(first, "client-a", 1, 101);
  first.operation = articore_wire_ACQUIRE_LEASE;
  check(dds_write(request_writer, &first), "write acquire request");
  const auto acquired = wait_reply(reply_reader, 101);
  require(acquired.error == articore_wire_OK && acquired.lease_id != 0,
          "first client receives a correlated lease");

  articore_wire_ControlRequest incompatible{};
  initialize(incompatible, "client-a", 2, 102);
  incompatible.protocol_major = 2;
  incompatible.operation = articore_wire_QUERY_HEALTH;
  check(dds_write(request_writer, &incompatible), "write incompatible request");
  const auto rejected = wait_reply(reply_reader, 102);
  require(rejected.error == articore_wire_VERSION_MISMATCH,
          "protocol major mismatch is rejected");

  articore_wire_ControlRequest competitor{};
  initialize(competitor, "client-b", 1, 103);
  competitor.operation = articore_wire_ACQUIRE_LEASE;
  check(dds_write(request_writer, &competitor), "write competing request");
  const auto busy = wait_reply(reply_reader, 103);
  require(busy.error == articore_wire_BUSY,
          "second DDS client cannot take the whole-robot lease");

  articore_wire_ControlRequest heartbeat{};
  initialize(heartbeat, "client-a", 100, 104);
  heartbeat.operation = articore_wire_HEARTBEAT;
  heartbeat.lease_id = acquired.lease_id;
  check(dds_write(request_writer, &heartbeat), "write heartbeat");
  require(wait_reply(reply_reader, 104).error == articore_wire_OK,
          "20 Hz heartbeat protocol renews the lease");

  // Control and stream use distinct DataWriters, so DDS does not promise
  // cross-topic delivery order. A high control sequence must never make a
  // valid independently ordered stream sequence stale. Waiting beyond the
  // heartbeat deadline proves that the accepted stream refreshed the lease.
  std::this_thread::sleep_for(180ms);
  articore_wire_StreamCommand stream{};
  stream.protocol_major = 1;
  stream.protocol_minor = 0;
  std::snprintf(stream.robot_id, sizeof(stream.robot_id), "%s",
                "loopback-robot");
  std::snprintf(stream.client_id, sizeof(stream.client_id), "%s", "client-a");
  stream.sequence_id = 1;
  stream.lease_id = acquired.lease_id;
  stream.kind = articore_wire_PV;
  stream.speed_percent = 20.0f;
  check(dds_write(stream_writer, &stream), "write independently sequenced PV");
  std::this_thread::sleep_for(150ms);

  articore_wire_ControlRequest after_stream{};
  initialize(after_stream, "client-a", 101, 105);
  after_stream.operation = articore_wire_CONFIGURE_MODE;
  after_stream.lease_id = acquired.lease_id;
  check(dds_write(request_writer, &after_stream),
        "write request after stream-refreshed lease");
  require(wait_reply(reply_reader, 105).error == articore_wire_TRANSPORT_ERROR,
          "stream remains authorized after a higher heartbeat sequence and "
          "refreshes the whole-robot lease");

  stop.store(true);
  service_thread.join();
  require(static_cast<bool>(service_status), "service shuts down cleanly");
  check(dds_delete(participant), "delete client participant");
}
