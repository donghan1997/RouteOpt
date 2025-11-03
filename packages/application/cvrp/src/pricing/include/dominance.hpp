/* 
 * Copyright (c) 2025 Zhengzhong (Ricky) You.
 * All rights reserved.
 * Software: RouteOpt
 * License: GPL-3.0
 */

#ifndef ROUTE_OPT_DOMINANCE_HPP
#define ROUTE_OPT_DOMINANCE_HPP
#include "pricing_macro.hpp"
#include "cvrp_pricing_controller.hpp"

namespace RouteOpt::Application::CVRP {
    template<bool dir>
    bool CVRP_Pricing::doRCTermDominance(Label *ki, Label *kj) {
        double gap = kj->rc - ki->rc;
        if (!rank1_rc_controller_ref.get().doR1CDominance(gap, ki->r1c, kj->r1c)) return false;
        return true;
    }

    template<bool dir, PRICING_LEVEL pricing_level>
    bool CVRP_Pricing::dominanceCore(Label *ki, Label *kj) {
        // std::vector<int> col_i{};
        // Label *p;
        // p = ki;
        // // std::cout << "ki end vertex : " << ki->end_vertex << " cost i = " << p->cost << std::endl;
        
        // while (p && p->end_vertex) {
        //     col_i.emplace_back(p->end_vertex);
        //     p = p->p_label;
        // }
        // std::reverse(col_i.begin(), col_i.end());
        // std::vector<int> col_j{};
        // p = kj;
        // // std::cout << "kj end vertex : " << kj->end_vertex <<  " cost j = " << p->cost << std::endl;

        // while (p && p->end_vertex) {
        //     col_j.emplace_back(p->end_vertex);
        //     p = p->p_label;
        // }
        // std::reverse(col_j.begin(), col_j.end());
        // // // print col _i and col_j
        // // std::cout << "col_i: ";
        // // for (auto &i : col_i) {
        // //     std::cout << i << " ";
        // // }
        // // std::cout << std::endl;
        // // std::cout << "col_j: ";
        // // for (auto &j : col_j) {
        // //     std::cout << j << " ";
        // // }
        // // std::cout << std::endl;

        // std::vector<int> fix_path = {13, 6, 15};
        // if (col_j.size() == fix_path.size()) {
        //     if (col_j == fix_path) {
                
        //         bool dominate = true;
        //         if (dir
        //             ? tellResTupleRelations<'p'>(ki->res, kj->res)
        //             : tellResTupleRelations<'p'>(kj->res, ki->res)) {
        //             dominate = false; 
        //         } 
        //         if (((ki->pi & kj->pi) ^ (ki->pi)).any()) {
        //             dominate = false;
        //         }
        //         // if (!doRCTermDominance<dir>(ki, kj)) return false;
        //         // if (!(kj->cost-ki->cost<1e-6)) return false;

        //         if (dominate) {

                    
                    
        //             double rc_gap = ki->rc - kj->rc;
        //             double cost_gap = kj->cost - ki->cost;
                    
        //             double prod1 = beta_min * cost_gap;
        //             double prod2 = beta_max * cost_gap;
        //             double real_rc_gap = rc_gap + (prod1 < prod2 ? prod2 : prod1);
        //             if (real_rc_gap >= -RC_TOLERANCE) std::cout << "real rc gap not hold" << std::endl;
        //             else {
        //                 std::cout << "$$$$$$$$$$$$ dominance hold $$$$$$$$$$$$" << std::endl;
        //                 std::cout << "col_i: ";
        //                 for (auto &i : col_i) {
        //                     std::cout << i << " ";
        //                 }
        //                 std::cout << std::endl;
        //                 std::cout << "col_j: ";
        //                 for (auto &j : col_j) {
        //                     std::cout << j << " ";
        //                 }
        //                 std::cout << std::endl;
        //                 std::cout << "ki rc: " << ki->rc << ", kj rc: " << kj->rc << std::endl;
        //                 std::cout << "ki cost: " << ki->cost << ", kj cost: " << kj->cost << std::endl;
        //                 std::cout << "rc_gap: " << rc_gap << ", cost_gap: " << cost_gap << std::endl;
        //                 std::cout << "prod1: " << prod1 << ", prod2: " << prod2 << std::endl;
        //                 std::cout << "real_rc_gap: " << real_rc_gap << std::endl;
        //                 // exit(0);
        //             }
        //         }
               
        //     }
        // }
        // exit(0);
        
        if constexpr (pricing_level == PRICING_LEVEL::EXACT) {

            // return false;
            //exact
            if (dir
                    ? tellResTupleRelations<'p'>(ki->res, kj->res)
                    : tellResTupleRelations<'p'>(kj->res, ki->res))
                return
                        false;
            if (((ki->pi & kj->pi) ^ (ki->pi)).any()) return false;
            // if (!doRCTermDominance<dir>(ki, kj)) return false;
            // if (!(kj->cost-ki->cost<1e-6)) return false;
            double rc_gap = ki->rc - kj->rc;
            double cost_gap = kj->cost - ki->cost;

            // 1. beta min and max 先算好，进来之后直接call，判断和RC的关系
            
            double prod1 = beta_min * cost_gap;
            double prod2 = beta_max * cost_gap;
            double real_rc_gap = rc_gap + (prod1 < prod2 ? prod2 : prod1);
            if (real_rc_gap >= -RC_TOLERANCE) return false;

            double range_gap = ki->cost < kj->cost ?  theta_max - ki->cost : theta_max - kj->cost;
            if (range_gap >= -RC_TOLERANCE) return false;

        } else if (pricing_level == PRICING_LEVEL::HEAVY) {
            if (dir
                    ? tellResTupleRelations<'p'>(ki->res, kj->res)
                    : tellResTupleRelations<'p'>(kj->res, ki->res))
                return
                        false;
        } else {
        }

        return true;
    }

    template<bool dir, PRICING_LEVEL pricing_level>
    void CVRP_Pricing::doDominance(Label *ki, int j, int bj, bool &if_suc) {
        auto &labelList_j = dir ? label_array_in_forward_sense[j][bj] : label_array_in_backward_sense[j][bj];
        auto new_label = all_label + idx_glo;
        auto &tmp_rc = new_label->rc;
        double len;
        // std::cout << "j = " << j << std::endl;

        auto &tmp_cost = new_label->cost;
        // std::cout << "tmp_cost = " << tmp_cost << std::endl;

        // std::cout << "end_vertex : " << new_label->end_vertex << std::endl;

        // std::vector<int> col_new{};
        // Label *p;
        // p = new_label;
        // // std::cout << "ki end vertex : " << ki->end_vertex << " cost i = " << p->cost << std::endl;
        
        // while (p && p->end_vertex) {
        //     col_new.emplace_back(p->end_vertex);
        //     p = p->p_label;
        // }
        // std::reverse(col_new.begin(), col_new.end());
        // std::cout << "col_new: ";
        // for (auto &i : col_new) {
        //     std::cout << i << " ";
        // }
        // std::cout << std::endl;

        double tmp_rc_add = tmp_rc + RC_TOLERANCE
                , tmp_rc_sub = tmp_rc - RC_TOLERANCE;
        if_suc = true;
        if constexpr (CHECK_PRICING_LABELS) len = 0;
        auto it = labelList_j.begin();
        for (; it != labelList_j.end(); ++it) {
            ++num_dominance_checks;
            if constexpr (CHECK_PRICING_LABELS) ++len;
            auto kj = *it;
            // std::vector<int> col_j{};
            // p = kj;
            // // std::cout << "kj end vertex : " << kj->end_vertex <<  " cost j = " << p->cost << std::endl;

            // while (p && p->end_vertex) {
            //     col_j.emplace_back(p->end_vertex);
            //     p = p->p_label;
            // }
            // std::reverse(col_j.begin(), col_j.end());
            // std::cout << "col_j: ";
            // for (auto &j : col_j) {
            //     std::cout << j << " ";
            // }
            // std::cout << std::endl;
            if (kj->rc < tmp_rc_add) {

                // std::cout << "(1) " << std::endl;

                if (dominanceCore<dir, pricing_level>(kj, new_label)) {
                HERE1:
                    // std::cout << "Here 1 " << std::endl;
                    labelList_j.splice(labelList_j.begin(), labelList_j, it);
                    if constexpr (CHECK_PRICING_LABELS) {
                        inner_bin_len.first += len;
                        inner_bin_len.second++;
                    }
                    if_suc = false;
                    return;
                }
            } 
            else if (kj->rc > tmp_rc_sub) {
            // else if (false){
                // std::cout << "(2) " << std::endl;
                if (dominanceCore<dir, pricing_level>(new_label, kj)) {
                HERE2:
                    // std::cout << "Here 2 " << std::endl;
                    kj->is_extended = true;
                    it = labelList_j.erase(it);
                    break;
                }
            } else {
                // std::cout << "(3) " << std::endl;
                if (dominanceCore<dir, pricing_level>(kj, new_label)) {
                    goto HERE1;
                } else if (dominanceCore<dir, pricing_level>(new_label, kj)) {
                    goto HERE2;
                }
            }
        }

        for (; it != labelList_j.end();) {
            auto kj = *it;
            if (kj->rc < tmp_rc_add) {
                ++it;
                continue;
            }
            if (dominanceCore<dir, pricing_level>(new_label, kj)) {
                // std::cout << "(4) " << std::endl;
                kj->is_extended = true;
                it = labelList_j.erase(it);
            } else ++it;
        }

        labelList_j.push_front(new_label);

        new_label->p_label = ki;
        new_label->is_extended = false;
        auto &bucket = dir
                           ? if_exist_extra_labels_in_forward_sense[j][bj]
                           : if_exist_extra_labels_in_backward_sense[j][bj];
        bucket.first[bucket.second++] = new_label;
        if (bucket.second == bucket.first.size()) {
            bucket.first.resize(bucket.first.size() * 2);
        }
    }

    template<bool dir, PRICING_LEVEL pricing_level>
    void CVRP_Pricing::checkIfDominated(Label *&ki, int i, int b,
                                        bool &if_suc) {
        if_suc = true;
        double tmp_ki_rc_sub = ki->rc - RC_TOLERANCE;
        // std::cout << "tmp_ki_rc_sub= " << tmp_ki_rc_sub << std::endl;
        int len;
        if constexpr (CHECK_PRICING_LABELS) len = 0;
        for (int b4_b = (dir ? b - 1 : b + 1); dir ? b4_b >= 0 : b4_b < num_buckets_per_vertex; dir ? --b4_b : ++b4_b) {
            // std::cout << "b4_b= " << b4_b << std::endl;
            auto &b4_label_list = dir ? label_array_in_forward_sense[i][b4_b] : label_array_in_backward_sense[i][b4_b];
            if ((dir ? rc2_till_this_bin_in_forward_sense[i][b4_b] : rc2_till_this_bin_in_backward_sense[i][b4_b])
                > tmp_ki_rc_sub)
                break;
            ++num_dominance_checks;
            for (auto &p: b4_label_list) {
                if constexpr (CHECK_PRICING_LABELS) ++len;
                if (p->rc > tmp_ki_rc_sub) break;
                // std::cout << "p->end_vertex = " << p->end_vertex
                //           << ", p->cost = " << p->cost << std::endl;
                if (dominanceCore<dir, pricing_level>(p, ki)) {
                    // std::cout << "checkIfDominated " << std::endl;
                    if_suc = false;
                    if constexpr (CHECK_PRICING_LABELS) {
                        outer_bin_len.first += len;
                        outer_bin_len.second++;
                    }
                    return;
                }
            }
        }
        if constexpr (CHECK_PRICING_LABELS) {
            outer_bin_but_keep_len.first += len;
            outer_bin_but_keep_len.second++;
        }
    }
}

#endif // ROUTE_OPT_DOMINANCE_HPP
