#include "lslidar_driver/flag_handler.hpp"

volatile sig_atomic_t flag = 1;

static void my_handler([[maybe_unused]] int sig)
{
  flag = 0;
}

void initialize_signal_handler()
{
  signal(SIGINT, my_handler);
}

