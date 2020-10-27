var led_status = 0;
exports.led_status = led_status;
exports.toggleLED = function () {
    if (!led_status)
    {
      led_status=1;
    }
    else
    {
      led_status=0
    }
}
