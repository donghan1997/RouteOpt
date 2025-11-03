/* 
 * Copyright (c) 2025 Zhengzhong (Ricky) You.
 * All rights reserved.
 * Software: RouteOpt
 * License: GPL-3.0
 */

#include "cuts_definition.hpp"
#include "cvrp_pricing_controller.hpp"
#include "rcc_rc_controller.hpp"

namespace RouteOpt::Application::CVRP {
    void CVRP_Pricing::priceConstraints(const std::vector<Rcc> &rccs,
                                        const std::vector<R1c> &r1cs,
                                        const std::vector<Brc> &brcs,
                                        const std::vector<double> &pi_vector) {
        pricePartitioning(pi_vector);
        pricePartitioningByCustomers(brcs, pi_vector);
        RCCs::RCGetter::RCCRCController::priceRCC(rccs, pi_vector, chg_cost_mat4_vertex);
        priceBRC(brcs, pi_vector);
        //price rank1 cuts;
        rank1_rc_controller_ref.get().getRank1DualsInCG(r1cs, pi_vector);
    }


    void CVRP_Pricing::pricePartitioning(const std::vector<double> &pi_vector) {
        auto &cm = cost_mat4_vertex_ref.get();
        // print cm
        // for (int i = 0; i < dim; ++i) {
        //     for (int j = 0; j < dim; ++j) {
        //         std::cout << cm[i][j] << " ";
        //     }
        //     std::cout << std::endl;
        // }
        // exit(0);

        // std::cout << SMALL_PHASE_SEPARATION;
        // for (int i = 0; i <= dim-2; ++i) {
        //     std::cout << pi_vector[i] << " ";
        // }
        // std::cout << std::endl;
        // for (int i = dim-1; i <= dim-1; ++i) {
        //     std::cout << pi_vector[i] << " ";
        // }
        // std::cout << std::endl;
        // for (int i = dim; i <= dim; ++i) {
        //     std::cout << pi_vector[i] << " ";
        // }
        // std::cout << std::endl;
        // for (int i = dim+1; i <= 2*dim-1; ++i) {
        //     std::cout << pi_vector[i] << " ";
        // }
        // std::cout << std::endl;
        // for (int i = 2*dim; i <= 3*dim-2; ++i) {
        //     std::cout << pi_vector[i] << " ";
        // }
        // std::cout << std::endl;

        auto real_dim = dim - 1;
        for (int i = 1; i < dim; ++i) {
            for (int j = i + 1; j < dim; ++j) {
                chg_cost_mat4_vertex[i][j] = - cm[i][j] * pi_vector[dim] - 0.5 * (pi_vector[i - 1] + pi_vector[j - 1]);
            }
        }
        for (int i = 1; i < dim; ++i) {
            chg_cost_mat4_vertex[0][i] = - cm[0][i] * pi_vector[dim]  - 0.5 * (pi_vector[i - 1] + pi_vector[real_dim]);
        }
        for (int i = 1; i < dim; ++i) {
            for (int j = i + 1; j < dim; ++j) {
                chg_cost_mat4_vertex[j][i] = chg_cost_mat4_vertex[i][j];
            }
        }
        for (int i = 1; i < dim; ++i) {
            chg_cost_mat4_vertex[i][0] = chg_cost_mat4_vertex[0][i];
        }

        // print chg_cost_mat4_vertex
        // std::cout << "chg_cost_mat4_vertex:" << std::endl;
        // for (int i = 0; i < dim; ++i) {
        //     for (int j = 0; j < dim; ++j) {
        //         std::cout << chg_cost_mat4_vertex[i][j] << " ";
        //     }
        //     std::cout << std::endl;
        // }
        
    }

    void CVRP_Pricing::pricePartitioningByCustomers(const std::vector<Brc> &brcs, const std::vector<double> &pi_vector){

        dual_vector = pi_vector;
        beta_max = -std::numeric_limits<double>::infinity();
        beta_min = std::numeric_limits<double>::infinity();
        
        for (int i = 1; i < dim; ++i) {
            double beta = dual_vector[dim + i] + dual_vector[2 * dim - 1 + i];
            if (beta > beta_max) beta_max = beta;
            if (beta < beta_min) beta_min = beta;
        }

        // std::cout << "beta_max = " << beta_max << ", beta_min = " << beta_min << std::endl;
        theta_max = 0.0;
        for (auto &brc: brcs) {
            if ((brc.edge.first==dim) && (brc.edge.second==dim)) {
                // if (brc.range.second > theta_max) theta_max = brc.range.second;
                if (brc.range.first == 1) { // m
                    if (brc.range.second > theta_max) theta_max = brc.range.second;
                }
                if (brc.range.first == 2) { // n
                    if (brc.range.second > theta_max) theta_max = brc.range.second;
                }
            }
        }


        brcs_from_node = brcs;
        

    }

    void CVRP_Pricing::priceBRC(const std::vector<Brc> &brcs, const std::vector<double> &pi_vector) {
        adjust_brc_dual4_single_route.clear();
        for (auto &brc: brcs) {
            if ((brc.edge.first==dim) && (brc.edge.second==dim)) continue; 
            if (!brc.br_dir) {
                chg_cost_mat4_vertex[brc.edge.first][brc.edge.second] = std::numeric_limits<float>::max();
                chg_cost_mat4_vertex[brc.edge.second][brc.edge.first] =
                        std::numeric_limits<float>::max(); //do not use double since the number will overflow
            } else {
                // in case multiple same branching constraints, we use += instead of just =;
                if (brc.edge.first == 0) adjust_brc_dual4_single_route[brc.edge.second] += pi_vector[brc.idx_brc];
                chg_cost_mat4_vertex[brc.edge.first][brc.edge.second] -= pi_vector[brc.idx_brc];
                chg_cost_mat4_vertex[brc.edge.second][brc.edge.first] -= pi_vector[brc.idx_brc];
            }
        }
    }
}
