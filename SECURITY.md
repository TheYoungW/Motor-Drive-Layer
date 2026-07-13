# Security and Safety Reporting

Motor-control defects can create physical safety risks. Do not open a public issue containing a turnkey procedure for uncontrolled motion, bypassing limits, defeating a watchdog, or remotely driving attached hardware.

Use GitHub's private vulnerability reporting feature for the repository when available. Include:

- affected release or commit;
- transport and adapter;
- motor model and firmware;
- whether motors must be enabled;
- minimum safe reproduction steps;
- expected versus observed fail-safe behavior;
- suggested mitigation, if known.

General bugs without a safety or security impact can use the public bug-report template.

The maintainers will acknowledge a private report, reproduce it in a safe environment where possible, and coordinate disclosure after a mitigation is available. No response-time guarantee is currently offered.
