#ifndef FLAG_HANDLER_HPP
#define FLAG_HANDLER_HPP

#include <csignal>

extern volatile sig_atomic_t flag;

void initialize_signal_handler();

#endif // FLAG_HANDLER_HPP

