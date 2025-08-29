#ifndef ROUTE_OPT_GLOBAL_CONFIG_HPP
#define ROUTE_OPT_GLOBAL_CONFIG_HPP

struct GlobalConfig {
    double BIG_M = 1e6;
    double Budget = 1e6;
    bool ALL_EDGES_IF_ONE = false;
};

extern GlobalConfig global_config;

#endif
