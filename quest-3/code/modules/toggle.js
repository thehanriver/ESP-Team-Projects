var led_status = 0;
exports.led_status = led_status;


export function toggleLED () {
    if (!led_status)
    {
      led_status=1;
    }
    else
    {
      led_status=0
    }
}
