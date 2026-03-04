#include <gpiod.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>

void print_time(const char* label)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);

    printf("%s: %ld.%09ld\n", label, ts.tv_sec, ts.tv_nsec);
}


int main() {

    const char *chipname = "/dev/gpiochip2";
    unsigned int line_num = 5;

    struct gpiod_chip *chip;
    struct gpiod_line *line;

    chip = gpiod_chip_open(chipname);
    if (!chip) {
        perror("Open chip failed");
        return 1;
    }

    line = gpiod_chip_get_line(chip, line_num);
    if (!line) {
        perror("Get line failed");
        return 1;
    }

    if (gpiod_line_request_output(line, "gpio_test", 0) < 0) {
        perror("Request line failed");
        return 1;
    }

    print_time("GPIO HIGH");
    gpiod_line_set_value(line, 1);

    usleep(50000);  // 50 ms

    print_time("GPIO LOW");
    gpiod_line_set_value(line, 0);

    gpiod_chip_close(chip);

    return 0;
}
