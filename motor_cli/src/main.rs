mod args;
mod damiao_cli;

use args::{get_str, get_u16_hex_or_dec, print_help};
use damiao_cli::run_damiao;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = args::parse_args();
    if args.is_empty() || args.contains_key("help") {
        print_help();
        return Ok(());
    }
    if args.contains_key("version") {
        println!("motor_cli {}", env!("CARGO_PKG_VERSION"));
        return Ok(());
    }

    let vendor = get_str(&args, "vendor", "damiao");
    let channel = get_str(&args, "channel", "can0");
    let transport = get_str(&args, "transport", "auto");
    let dm_device_type = get_str(&args, "dm-device-type", "usb2canfd-dual");
    if vendor != "damiao" {
        return Err(format!("unsupported vendor in this Damiao-only fork: {vendor}").into());
    }
    let model = get_str(&args, "model", "4340");
    let motor_id = get_u16_hex_or_dec(&args, "motor-id", 0x01)?;
    let feedback_id = get_u16_hex_or_dec(&args, "feedback-id", 0x0011)?;
    let mode = get_str(
        &args,
        "mode",
        "mit",
    );
    let dm_channel = if mode == "scan" && transport == "dm-device" {
        args.get("dm-channel")
            .cloned()
            .unwrap_or_else(|| "all".to_string())
    } else {
        get_str(&args, "dm-channel", "0")
    };

    let model_is_default = !args.contains_key("model");
    let motor_id_is_default = !args.contains_key("motor-id");
    let feedback_id_is_default = !args.contains_key("feedback-id");
    let start_id_is_default = !args.contains_key("start-id");
    let end_id_is_default = !args.contains_key("end-id");
    let default_tag = |is_default: bool| if is_default { " (default)" } else { "" };

    if mode == "scan" {
        let scan_start = get_u16_hex_or_dec(&args, "start-id", 1)?;
        let scan_end = get_u16_hex_or_dec(&args, "end-id", 255)?;
        println!(
            "vendor={} transport={} channel={}{} mode=scan model_hint={}{} base_feedback_id=0x{:X}{} scan_range={}{}..{}{}",
            vendor,
            transport,
            channel,
            if transport == "dm-device" {
                format!(" dm_device_type={} dm_channel={}", dm_device_type, dm_channel)
            } else {
                String::new()
            },
            model,
            default_tag(model_is_default),
            feedback_id,
            default_tag(feedback_id_is_default),
            scan_start,
            default_tag(start_id_is_default),
            scan_end,
            default_tag(end_id_is_default),
        );
    } else {
        println!(
            "vendor={} transport={} channel={}{} model={}{} motor_id=0x{:X}{} feedback_id=0x{:X}{} mode={}",
            vendor,
            transport,
            channel,
            if transport == "dm-device" {
                format!(" dm_device_type={} dm_channel={}", dm_device_type, dm_channel)
            } else {
                String::new()
            },
            model,
            default_tag(model_is_default),
            motor_id,
            default_tag(motor_id_is_default),
            feedback_id,
            default_tag(feedback_id_is_default),
            mode
        );
    }

    if transport == "dm-serial" && vendor != "damiao" {
        return Err("transport=dm-serial is currently supported only for --vendor damiao".into());
    }
    if transport == "dm-device" && vendor != "damiao" {
        return Err("transport=dm-device is currently supported only for --vendor damiao".into());
    }

    run_damiao(&args, &channel, &model, motor_id, feedback_id)
}
