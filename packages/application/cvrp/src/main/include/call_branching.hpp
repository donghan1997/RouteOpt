/* 
 * Copyright (c) 2025 Zhengzhong (Ricky) You.
 * All rights reserved.
 * Software: RouteOpt
 * License: GPL-3.0
 */

#ifndef ROUTE_OPT_CALL_BRANCHING_HPP
#define ROUTE_OPT_CALL_BRANCHING_HPP
#include <numeric>
#include <route_opt_macro.hpp>
#include "global_config.hpp"
#include "candidate_selector_macro.hpp"

namespace RouteOpt::Application::CVRP {
    namespace TestingDetail {
        inline void addBranchConstraint(
            std::vector<int> &cind,
            std::vector<double> &cval,
            Solver &node_solver, bool dir = false) {
            if (dir) {
                if (cind.front() == 0) {
                    cval.front() = 1;
                } else {
                    cind.emplace(cind.begin(), 0);
                    cval.emplace(cval.begin(), 1);
                }
                // print the branch constraint
                // std::cout << "branch constraint: ";
                // for (const auto &i: cind) {
                //     std::cout << i << " ";
                // }
                // std::cout << std::endl;
                SAFE_SOLVER(node_solver.addConstraint(cind.size(), cind.data(), cval.data(), SOLVER_EQUAL, 1, nullptr))
            } else {
                if (cind.front() == 0) {
                    cind.erase(cind.begin());
                    cval.erase(cval.begin());
                }
                SAFE_SOLVER(node_solver.addConstraint(cind.size(), cind.data(), cval.data(), SOLVER_EQUAL, 0, nullptr))
            }
            SAFE_SOLVER(node_solver.updateModel())
        }


        inline void addCustomerBranchConstraint(
            std::vector<int> &cind,
            std::vector<double> &cval,
            Solver &node_solver, bool dir = false) {   
            
            // print the branch constraint
            // std::cout << "branch constraint: ";
            // for (const auto &i: cind) {
            //     std::cout << i << " ";
            // }
            // std::cout << std::endl;
            if (dir) {
                if (cind.front() == 0) {
                cval.front() = 1;
                } else {
                    cind.emplace(cind.begin(), 0);
                    cval.emplace(cval.begin(), 1);
                }
                SAFE_SOLVER(node_solver.addConstraint(cind.size(), cind.data(), cval.data(), SOLVER_EQUAL, 1, nullptr))
            } else {
                if (cind.front() == 0) {
                    cind.erase(cind.begin());
                    cval.erase(cval.begin());
                }
                SAFE_SOLVER(node_solver.addConstraint(cind.size(), cind.data(), cval.data(), SOLVER_EQUAL, 0, nullptr))
            }
            SAFE_SOLVER(node_solver.updateModel())
        }

        inline void addRangeBranchConstraint(int row_idx, double value, Solver &node_solver){

            // std::cout << "add range branch constraint at row " << row_idx << " with value " << value << std::endl;
            std::vector<double> values(1, value);
            SAFE_SOLVER(node_solver.setRhs(row_idx, 1, values.data()))
            SAFE_SOLVER(node_solver.updateModel())
        }

        inline void addRangeConstraint(std::vector<int> &cind,
            std::vector<double> &cval,
            Solver &node_solver) {

            SAFE_SOLVER(node_solver.addConstraint(cind.size(), cind.data(), cval.data(), SOLVER_EQUAL, 0, nullptr))
            SAFE_SOLVER(node_solver.updateModel())

        }


        inline void inverseLastBranchConstraint(Solver &node_solver) {
            SAFE_SOLVER(node_solver.updateModel())
            int num_row;
            SAFE_SOLVER(node_solver.getNumRow(&num_row))
            --num_row;
            double rhs = 1.;
            SAFE_SOLVER(node_solver.setRhs(num_row, 1,&rhs))
            int vind = 0;
            SAFE_SOLVER(node_solver.changeCoeffs(1, &num_row, &vind, &rhs))
            SAFE_SOLVER(node_solver.updateModel())
        }

        inline double calculateDifference(double tmp_val, double prior_val) {
            return std::max(tmp_val - prior_val, TOLERANCE);
        }

        template<bool if_exact>
        void callLabelingInTesting(CVRPSolver *cvrp, BbNode *node) {
            constexpr bool if_open_heur = true;
            constexpr bool if_open_exact = if_exact;
            constexpr bool if_update_node_val = false;
            constexpr bool if_possible_terminate_early = true;
            constexpr bool if_fix_row = true;
            constexpr double labeling_time_limit = std::numeric_limits<float>::max();

            constexpr bool if_consider_regenerate_bucket_graph = false;
            constexpr bool if_fix_meet_point = true;
            constexpr bool if_allow_delete_col = false;

            cvrp->solveLPInLabeling(node, if_open_heur, if_open_exact, if_update_node_val,
                                    if_consider_regenerate_bucket_graph, if_possible_terminate_early,
                                    if_fix_row, if_fix_meet_point, if_allow_delete_col, labeling_time_limit);
            node->refIfTerminate() = false;
        };

        inline void callInspectionInTesting(CVRPSolver *cvrp, BbNode *node) {
            constexpr bool if_update_column_pool = false;
            constexpr bool if_allow_delete_col = false;
            cvrp->solveLPByInspection(node, if_update_column_pool, if_allow_delete_col);
        }

        template<bool if_exact>
        void callPricingInTesting(CVRPSolver *cvrp, BbNode *node, double &tmp_val) {
            int num_col;
            SAFE_SOLVER(node->refSolver().getNumCol(&num_col))
            // std::cout << "callPricingInTesting, num_cols = " << num_col << std::endl;
            if (node->getIfInEnumState()) {
                callInspectionInTesting(cvrp, node);
            } else {
                callLabelingInTesting<if_exact>(cvrp, node);
            }
            SAFE_SOLVER(node->refSolver().getObjVal(&tmp_val))
            int new_num_col;
            SAFE_SOLVER(node->refSolver().getNumCol(&new_num_col))
            // std::cout << "After callPricingInTesting, num_cols = " << new_num_col << std::endl;

            std::vector<int> col_idx(new_num_col - num_col);
            std::iota(col_idx.begin(), col_idx.end(), num_col);
            node->rmLPCols(col_idx);
        }
    }

    template<bool if_exact>
    void CVRPSolver::processCGTesting(BbNode *node, const std::pair<int, int> &edge, double &dif1, double &dif2) {

        auto &node_solver = node->refSolver();
        double tmp_val, org_val = node->getValue();

        int b4_num_row;
        SAFE_SOLVER(node_solver.getNumRow(&b4_num_row))

        if (edge.first == edge.second) {


            int num_col;
            SAFE_SOLVER(node_solver.getNumCol(&num_col))


            std::vector<int> cbeg;
            std::vector<int> cind;
            std::vector<double> cval;
            int numnzP;
            int start = node->getDim();
            SAFE_SOLVER(node_solver.getConstraints(&numnzP, nullptr, nullptr, nullptr, start, 1))
            cbeg.resize(numnzP+1);
            cind.resize(numnzP);
            cval.resize(numnzP);
            SAFE_SOLVER(node_solver.getConstraints(&numnzP, cbeg.data(), cind.data(), cval.data(), start, 1))

            std::vector<double> rhs_m_n(4);
            SAFE_SOLVER(node_solver.getRhs(3*start-1+1, 4, rhs_m_n.data()))
            double lb_m = rhs_m_n[0];
            double ub_m = rhs_m_n[1];
            double lb_n = rhs_m_n[2];
            double ub_n = rhs_m_n[3];
            
            if (node->getBranchOnM()){
                double upper_m = node->getBigU();
                // m >= LB
                TestingDetail::addRangeBranchConstraint(3*start-1+1, static_cast<int>(upper_m)+1, node_solver);
                TestingDetail::callPricingInTesting<if_exact>(this, node, tmp_val);


                dif1 = TestingDetail::calculateDifference(tmp_val, org_val);

                // double rhs;
                SAFE_SOLVER(node_solver.setRhs(3*start, 1, &lb_m))
                SAFE_SOLVER(node_solver.updateModel())

                // m <= UB
                node->refBrCs().back().br_dir = true;
                TestingDetail::addRangeBranchConstraint(3*start+1, static_cast<int>(upper_m), node_solver);
                std::vector<int> solver_ind;
                std::vector<double> solver_val;
                for (int i = 3; i < num_col; ++i) {
                    if (cval[i-2] - static_cast<int>(upper_m) >= TOLERANCE) {
                        solver_ind.emplace_back(i);
                        solver_val.emplace_back(1.0);
                    }
                }
                TestingDetail::addRangeConstraint(solver_ind, solver_val, node_solver);
                TestingDetail::callPricingInTesting<if_exact>(this, node, tmp_val);
                dif2 = TestingDetail::calculateDifference(tmp_val, org_val);

                SAFE_SOLVER(node_solver.delConstraints(1, &b4_num_row))
                SAFE_SOLVER(node_solver.setRhs(3*start+1, 1, &ub_m))
                SAFE_SOLVER(node_solver.updateModel())
                node->refBrCs().pop_back();    
            
            }
            else if (node->getBranchOnN()) {
                double upper_n = node->getBigL();
                double lower_n = std::ceil(upper_n);

                // n >= LB
                node->refBrCs().emplace_back(Brc{edge, b4_num_row, true});
                TestingDetail::addRangeBranchConstraint(3*start+1+1, static_cast<int>(lower_n), node_solver);
                std::vector<int> solver_ind;
                std::vector<double> solver_val;
                for (int i = 3; i < num_col; ++i) {
                    if (cval[i-2] - static_cast<int>(lower_n) <= TOLERANCE) {
                        solver_ind.emplace_back(i);
                        solver_val.emplace_back(1.0);
                    }
                }
                TestingDetail::addRangeConstraint(solver_ind, solver_val, node_solver);
                TestingDetail::callPricingInTesting<if_exact>(this, node, tmp_val);

                dif1 = TestingDetail::calculateDifference(tmp_val, org_val);

                SAFE_SOLVER(node_solver.delConstraints(1, &b4_num_row))
                SAFE_SOLVER(node_solver.setRhs(3*start+2, 1, &lb_n))
                SAFE_SOLVER(node_solver.updateModel())
                
                // n <= UB
                node->refBrCs().back().br_dir = false;
                TestingDetail::addRangeBranchConstraint(3*start+2+1, static_cast<int>(lower_n)-1, node_solver);
                TestingDetail::callPricingInTesting<if_exact>(this, node, tmp_val);
                
                dif2 = TestingDetail::calculateDifference(tmp_val, org_val);

                // double rhs;
                SAFE_SOLVER(node_solver.setRhs(3*start+3, 1, &ub_n))
                SAFE_SOLVER(node_solver.updateModel())
                node->refBrCs().pop_back();

            }
            else {
                std::cout << "no branching on m and n" << std::endl;
                return;
            }


        }
        else {
            std::cout << "evaluate on edge: " << edge.first << "-" << edge.second << std::endl;
            // auto &node_solver = node->refSolver();

            // double tmp_val, org_val = node->getValue();
            std::vector<int> solver_ind;
            std::vector<double> solver_val;
            node->obtainBrcCoefficient(edge, solver_ind, solver_val);

            // int b4_num_row;
            // SAFE_SOLVER(node_solver.getNumRow(&b4_num_row))

            if (node->getIfInEnumState())
                node->addBranchConstraint2ColPoolInEnumByColMap(
                    edge, pricing_controller.getColumnPoolPtr());

            node->refBrCs().emplace_back(Brc{edge, b4_num_row, false});
            TestingDetail::addBranchConstraint(solver_ind, solver_val, node_solver);
            TestingDetail::callPricingInTesting<if_exact>(this, node, tmp_val);
            dif1 = TestingDetail::calculateDifference(tmp_val, org_val);

            node->refBrCs().back().br_dir = true;
            TestingDetail::inverseLastBranchConstraint(node_solver);
            TestingDetail::callPricingInTesting<if_exact>(this, node, tmp_val);
            dif2 = TestingDetail::calculateDifference(tmp_val, org_val);

            SAFE_SOLVER(node_solver.delConstraints(1, &b4_num_row))
            SAFE_SOLVER(node_solver.updateModel())
            node->refBrCs().pop_back();
            if (node->getIfInEnumState())node->refMatrixColPool().pop_back();

        }
    }


    template<bool if_exact>
    void CVRPSolver::processOneSideCGTesting(BbNode *node, const std::pair<int, int> &edge, double &dif1,
                                             bool dir) {
        auto &node_solver = node->refSolver();

        double tmp_val, org_val = node->getValue();
        std::vector<int> solver_ind;
        std::vector<double> solver_val;
        node->obtainBrcCoefficient(edge, solver_ind, solver_val);

        int b4_num_row;
        SAFE_SOLVER(node_solver.getNumRow(&b4_num_row))

        if (node->getIfInEnumState())
            node->addBranchConstraint2ColPoolInEnumByColMap(
                edge, pricing_controller.getColumnPoolPtr());

        node->refBrCs().emplace_back(Brc{edge, b4_num_row, dir});
        TestingDetail::addBranchConstraint(solver_ind, solver_val, node_solver, dir);
        TestingDetail::callPricingInTesting<if_exact>(this, node, tmp_val);
        dif1 = TestingDetail::calculateDifference(tmp_val, org_val);

        SAFE_SOLVER(node_solver.delConstraints(1, &b4_num_row))
        SAFE_SOLVER(node_solver.updateModel())
        node->refBrCs().pop_back();
        if (node->getIfInEnumState())node->refMatrixColPool().pop_back();
    }

    inline void CVRPSolver::processLPTesting(BbNode *node, const std::pair<int, int> &edge, double &dif1,
                                             double &dif2) {
        
        auto &node_solver = node->refSolver();
        SAFE_SOLVER(node_solver.setEnvCrossOver(SOLVER_CROSSOVER_DOWN))
        // SAFE_SOLVER(node_solver.setEnvOutputFlag(1, false))

        double tmp_val, org_val = node->getValue();
        int num_row;
        SAFE_SOLVER(node_solver.getNumRow(&num_row))                                        
        if (edge.first == edge.second) {

            int num_col;
            SAFE_SOLVER(node_solver.getNumCol(&num_col))

            std::vector<int> cbeg;
            std::vector<int> cind;
            std::vector<double> cval;
            int numnzP;
            int start = node->getDim();
            SAFE_SOLVER(node_solver.getConstraints(&numnzP, nullptr, nullptr, nullptr, start, 1))
            cbeg.resize(numnzP+1);
            cind.resize(numnzP);
            cval.resize(numnzP);
            SAFE_SOLVER(node_solver.getConstraints(&numnzP, cbeg.data(), cind.data(), cval.data(), start, 1))

            
            bool branch_on_m = node->getBranchOnM();
            bool branch_on_n = node->getBranchOnN();

            bool branch_on_customer = node->getBranchOnCustomer();

            if (!branch_on_m && !branch_on_n && branch_on_customer) {

                node->setBranchOnCustomer(false);
                node->setBranchCustomerIdx(-1);

                std::vector<double> xval(num_col);
                SAFE_SOLVER(node_solver.getX(0, num_col, xval.data()))
                int dim = node->getDim();
                const auto &col = node->getCols();

                // for different customer do the branching
                std::vector<double> customer_cost_contribution_m(dim, 0.0);
                std::vector<double> customer_cost_contribution_n(dim, 0.0);
                std::vector<double> customer_x_contribution(dim, 0.0);

                for (int i = 3; i < num_col; ++i) {
                    if (xval[i] > SOL_X_TOLERANCE) {
                        auto &ci = col[i-2];
                        auto &seq = ci.col_seq;
                        double cost = cval[i-2];
                        // find the last customer in the sequence
                        if (!seq.empty()) {
                            int last_customer = seq.back();
                            customer_cost_contribution_m[last_customer] += cost * xval[i];
                            customer_cost_contribution_n[last_customer] += (cost - global_config.BIG_M) * xval[i];
                            customer_x_contribution[last_customer] += xval[i];
                        }
                    }
                }

                // sort customer_x_contribution
                std::vector<int> cus_idx(dim-1);
                std::iota(cus_idx.begin(), cus_idx.end(), 1);
                std::sort(cus_idx.begin(), cus_idx.end(), [&](int a, int b) {
                    return customer_x_contribution[a] > customer_x_contribution[b];
                });

                int test_size = std::min(3, dim -1);
                int cnt_tested = 0;
                double max_abs_diff = -1.0;
                for (int i = 0; i < cus_idx.size(); ++i) {
                    if (equalFloat(customer_x_contribution[cus_idx[i]], 1., TOLERANCE)) continue;
                    if (equalFloat(customer_x_contribution[cus_idx[i]], 0., TOLERANCE)) continue;


                    int branch_customer_idx = cus_idx[i];
                    ++cnt_tested;
                    if (cnt_tested > test_size) break;

                    std::vector<int> solver_ind;
                    std::vector<double> solver_val;

                    for (int i = 1; i < col.size(); ++i) {
                        if (col[i].col_seq.back() == branch_customer_idx) {
                            solver_ind.emplace_back(i + 2);
                            solver_val.emplace_back(1.0);
                        }
                    }
                    TestingDetail::addCustomerBranchConstraint(solver_ind, solver_val, node_solver, true);
                    SAFE_SOLVER(node_solver.reoptimize(SOLVER_BARRIER))
                    SAFE_SOLVER(node_solver.getObjVal(&tmp_val))

                    dif1 = TestingDetail::calculateDifference(tmp_val, org_val);
                    SAFE_SOLVER(node_solver.delConstraints(1, &num_row))
                    SAFE_SOLVER(node_solver.updateModel())

                    TestingDetail::addCustomerBranchConstraint(solver_ind, solver_val, node_solver, false);
                    SAFE_SOLVER(node_solver.reoptimize(SOLVER_BARRIER))
                    SAFE_SOLVER(node_solver.getObjVal(&tmp_val))


                    dif2 = TestingDetail::calculateDifference(tmp_val, org_val);
                    SAFE_SOLVER(node_solver.delConstraints(1, &num_row))
                    SAFE_SOLVER(node_solver.updateModel())
                    // SAFE_SOLVER(node_solver.setEnvCrossOver(SOLVER_CROSSOVER_DEFAULT))

                    // std::cout << "branching on customer " << branch_customer_idx << ", dif1 = " << dif1 << ", dif2 = " << dif2 << std::endl;

                    double current_abs_diff = std::abs(dif1 - dif2);
                    if (current_abs_diff > max_abs_diff) {
                        max_abs_diff = current_abs_diff;
                        node->setBranchOnCustomer(true);
                        node->setBranchCustomerIdx(branch_customer_idx);
                        // std::cout << "Decide to branch on customer " << branch_customer_idx << std::endl;
                    }

                }

                SAFE_SOLVER(node_solver.setEnvCrossOver(SOLVER_CROSSOVER_DEFAULT))

                return;
 
                
            }


            std::vector<double> rhs_m_n(4);
            SAFE_SOLVER(node_solver.getRhs(3*start-1+1, 4, rhs_m_n.data()))
            double lb_m = rhs_m_n[0];
            double ub_m = rhs_m_n[1];
            double lb_n = rhs_m_n[2];
            double ub_n = rhs_m_n[3];
            
            // Here we need to check different alpha values
            std::vector<double> alpha_list = {0.01, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5};
            
            double UB_L = node->getBigU();
            double LB_U = node->getBigL();

            for (const auto &alpha : alpha_list){

                // std::cout << "Testing with alpha = " << alpha << std::endl;
                double big_U = (1 + alpha) * UB_L;
                double big_L = (1 - alpha) *  LB_U;

                if ((big_U - lb_m <= TOLERANCE) || (big_U - ub_m >= TOLERANCE)) branch_on_m = false;
                if ((big_L - lb_n <= TOLERANCE) || (big_L - ub_n >= TOLERANCE)) branch_on_n = false;

                if (!branch_on_m && !branch_on_n) {
                    std::cout << "Both big U and big L are out of range, no need to test" << std::endl;
                    continue;
                }
                else {
                    if (branch_on_m){

                        
                        double upper_m = big_U;

                        // m >= LB
                        TestingDetail::addRangeBranchConstraint(3*start-1+1, static_cast<int>(upper_m)+1, node_solver);

                        SAFE_SOLVER(node_solver.reoptimize(SOLVER_BARRIER))
                        SAFE_SOLVER(node_solver.getObjVal(&tmp_val))

                        dif1 = TestingDetail::calculateDifference(tmp_val, org_val);

                        // double rhs;
                        SAFE_SOLVER(node_solver.setRhs(3*start, 1, &lb_m))
                        SAFE_SOLVER(node_solver.updateModel())

                        // m <= UB
                        TestingDetail::addRangeBranchConstraint(3*start+1, static_cast<int>(upper_m), node_solver);
                        std::vector<int> solver_ind;
                        std::vector<double> solver_val;
                        for (int i = 3; i < num_col; ++i) {
                            if (cval[i-2] - static_cast<int>(upper_m) >= TOLERANCE) {
                                solver_ind.emplace_back(i);
                                solver_val.emplace_back(1.0);
                            }
                        }
                        TestingDetail::addRangeConstraint(solver_ind, solver_val, node_solver);
                        
                        SAFE_SOLVER(node_solver.reoptimize(SOLVER_BARRIER))
                        SAFE_SOLVER(node_solver.getObjVal(&tmp_val))
                        dif2 = TestingDetail::calculateDifference(tmp_val, org_val);


                        SAFE_SOLVER(node_solver.delConstraints(1, &num_row))
                        SAFE_SOLVER(node_solver.setRhs(3*start+1, 1, &ub_m))
                        SAFE_SOLVER(node_solver.updateModel())
                        // SAFE_SOLVER(node_solver.setEnvCrossOver(SOLVER_CROSSOVER_DEFAULT))

                        // std::cout << "branch_on_m = true, " << "big_U = " << big_U << ", m >= LB, dif1 = " << dif1 << ", m <= UB, dif2 = " << dif2 << std::endl;
                        if (dif1 + dif2 > 1){
                            node->setBranchOnM(branch_on_m);
                            node->setBigU(big_U);
                        }
                    }

                    if (branch_on_n) {
                        double upper_n = big_L;
                        double lower_n = std::ceil(upper_n);

                        // n <= UB
                        TestingDetail::addRangeBranchConstraint(3*start+2+1, static_cast<int>(lower_n)-1, node_solver);

                        SAFE_SOLVER(node_solver.reoptimize(SOLVER_BARRIER))
                        SAFE_SOLVER(node_solver.getObjVal(&tmp_val))

                        dif1 = TestingDetail::calculateDifference(tmp_val, org_val);

                        // double rhs;
                        SAFE_SOLVER(node_solver.setRhs(3*start+3, 1, &ub_n))
                        SAFE_SOLVER(node_solver.updateModel())

                        // n >= LB
                        TestingDetail::addRangeBranchConstraint(3*start+1+1, static_cast<int>(lower_n), node_solver);
                        std::vector<int> solver_ind;
                        std::vector<double> solver_val;
                        for (int i = 3; i < num_col; ++i) {
                            if (cval[i-2] - static_cast<int>(lower_n) <= TOLERANCE) {
                                solver_ind.emplace_back(i);
                                solver_val.emplace_back(1.0);
                            }
                        }
                        TestingDetail::addRangeConstraint(solver_ind, solver_val, node_solver);
                        
                        SAFE_SOLVER(node_solver.reoptimize(SOLVER_BARRIER))
                        SAFE_SOLVER(node_solver.getObjVal(&tmp_val))
                        dif2 = TestingDetail::calculateDifference(tmp_val, org_val);


                        SAFE_SOLVER(node_solver.delConstraints(1, &num_row))
                        SAFE_SOLVER(node_solver.setRhs(3*start+2, 1, &lb_n))
                        SAFE_SOLVER(node_solver.updateModel())
                        // SAFE_SOLVER(node_solver.setEnvCrossOver(SOLVER_CROSSOVER_DEFAULT))

                        // std::cout << "branch_on_n = true, " << "big_L = " << big_L << ", n <= UB, dif1 = " << dif1 << ", n >= LB, dif2 = " << dif2 << std::endl;
                        if (dif1 + dif2 > 1){
                            node->setBranchOnN(branch_on_n);
                            node->setBigL(big_L);
                        }

                    }

                    
                }
                
            }

            SAFE_SOLVER(node_solver.setEnvCrossOver(SOLVER_CROSSOVER_DEFAULT))

            // exit(0);



            



            
        }
        else{
            // auto &node_solver = node->refSolver();
            // SAFE_SOLVER(node_solver.setEnvCrossOver(SOLVER_CROSSOVER_DOWN))

            // double tmp_val, org_val = node->getValue();
            std::vector<int> solver_ind;
            std::vector<double> solver_val;
            node->obtainBrcCoefficient(edge, solver_ind, solver_val);

            // int num_row;
            // SAFE_SOLVER(node_solver.getNumRow(&num_row))

            TestingDetail::addBranchConstraint(solver_ind, solver_val, node_solver);

            SAFE_SOLVER(node_solver.reoptimize(SOLVER_BARRIER))
            SAFE_SOLVER(node_solver.getObjVal(&tmp_val))

            dif1 = TestingDetail::calculateDifference(tmp_val, org_val);

            if constexpr (ml_type != ML_TYPE::ML_NO_USE) {
                l2b_controller.collectResolvingDualRC(node->refSolver(), edge, num_row, true);
            }

            TestingDetail::inverseLastBranchConstraint(node_solver);

            SAFE_SOLVER(node_solver.reoptimize(SOLVER_BARRIER))
            SAFE_SOLVER(node_solver.getObjVal(&tmp_val))
            dif2 = TestingDetail::calculateDifference(tmp_val, org_val);

            if constexpr (ml_type != ML_TYPE::ML_NO_USE) {
                l2b_controller.collectResolvingDualRC(node->refSolver(), edge, num_row, false);
            }

            SAFE_SOLVER(node_solver.delConstraints(1, &num_row))
            SAFE_SOLVER(node_solver.updateModel())
            SAFE_SOLVER(node_solver.setEnvCrossOver(SOLVER_CROSSOVER_DEFAULT))

        }
        
    };
    
    inline void BbNode::obtainBrcMap() {

        // std::cout << "number of routes : " << cols.size() << std::endl;
        if (!edge_col_map.empty()) return;
        // std::unordered_map<std::pair<int, int>, std::vector<std::pair<int, int> > > edge_col_map{}; //edge, col idx, cnt
        for (int i = 0; i < static_cast<int>(cols.size()); ++i) {
            const auto &c = cols[i].col_seq;
            if (c.size() == 1) {
                edge_col_map[{0, c[0]}].emplace_back(i, 1);
                continue;
            }
            int b4 = 0;
            for (int j: c) {
                std::pair<int, int> key = (b4 < j) ? std::make_pair(b4, j) : std::make_pair(j, b4);
                auto &vec = edge_col_map[key];
                if (!vec.empty() && vec.back().first == i) {
                    ++vec.back().second;
                } else {
                    vec.emplace_back(i, 1);
                }
                b4 = j;
            }
            auto &vec = edge_col_map[{0, b4}];
            if (!vec.empty() && vec.back().first == i) {
                ++vec.back().second;
            } else {
                vec.emplace_back(i, 1);
            }
        }
    }

    inline void BbNode::addBranchConstraint2ColPoolInEnumByColMap(const std::pair<int, int> &edge,
                                                                  const int *col_pool4_pricing) {
        auto size = index_columns_in_enumeration_column_pool.size();
        if (size == 0) return;
        sparseRowMatrixXd mat_last(1, size);
        mat_last.setZero();
        std::vector<Eigen::Triplet<double> > triplets(size);
        sparseRowMatrixXd tmp(1, size);
        int cnt = 0;
        int ai = edge.first, aj = edge.second;
        auto &mat0 = matrix_in_enumeration.front();
        if (ai) {
            tmp = mat0.row(ai - 1) + mat0.row(aj - 1);
            for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(tmp, 0); it; ++it) {
                if (it.value() > 1.5) {
                    for (auto j = index_columns_in_enumeration_column_pool[it.col()] + 1;; ++j) {
                        int current_node = col_pool4_pricing[j];
                        if (!current_node) break;
                        if (current_node == ai) {
                            if (col_pool4_pricing[j + 1] == aj || col_pool4_pricing[j - 1] == aj)
                                triplets[cnt++] = {0, static_cast<int>(it.col()), 1};
                        }
                    }
                }
            }
        } else {
            for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(mat0, aj - 1); it; ++it) {
                if (it.value() > 0.5) {
                    auto j = index_columns_in_enumeration_column_pool[it.col()] + 1;
                    if (col_pool4_pricing[j] == aj) {
                        triplets[cnt++] = {0, static_cast<int>(it.col()), 1};
                        continue;
                    }
                    for (;; ++j) {
                        if (int current_node = col_pool4_pricing[j]; !current_node) break;
                    }
                    if (col_pool4_pricing[j - 1] == aj) {
                        triplets[cnt++] = {0, static_cast<int>(it.col()), 1};
                    }
                }
            }
        }
        SAFE_EIGEN(mat_last.setFromTriplets(triplets.begin(), triplets.end());)
        matrix_in_enumeration.push_back(std::move(mat_last));
    }


    inline void BbNode::obtainBrcCoefficient(const std::pair<int, int> &edge, std::vector<int> &ind,
                                             std::vector<double> &val) {
        obtainBrcMap();
        auto &map_ = edge_col_map;
        int size = static_cast<int>(map_[edge].size());
        ind.resize(size);
        val.resize(size);
        int cnt = 0;
        for (auto &col: map_[edge]) {
            // ind[cnt] = col.first;
            ind[cnt] = col.first > 0 ? col.first + 2 : col.first;
            val[cnt++] = col.second;
        }
    }


    inline void BbNode::obtainColIdxNotAllowByEdge(const std::pair<int, int> &edge, std::vector<int> &col_idx) {
        obtainBrcMap();
        auto &map_ = edge_col_map;
        auto [ai, aj] = edge;

        int num_col;
        SAFE_SOLVER(solver.getNumCol(&num_col))
        std::vector<int> beg(2);
        std::vector<int> ind(num_col);
        std::vector<double> val(num_col);

        std::unordered_set<int> st_;
        st_.reserve(num_col);
        int nz;
        if (ai != 0) {
            SAFE_SOLVER(solver.getConstraints(&nz,
                beg.data(),
                ind.data(),
                val.data(),
                ai - 1,
                1))
            st_.insert(ind.begin(), ind.begin() + nz);
        }
        SAFE_SOLVER(solver.getConstraints(&nz,
            beg.data(),
            ind.data(),
            val.data(),
            aj - 1,
            1))
        st_.insert(ind.begin(), ind.begin() + nz);
        for (auto &col: map_[edge]) {
            st_.erase(col.first);
        }
        col_idx.assign(st_.begin(), st_.end());
        std::sort(col_idx.begin(), col_idx.end());
    }


    inline std::unordered_map<std::pair<int, int>, double, PairHasher> BbNode::obtainSolEdgeMap(BbNode *node) {
        std::unordered_map<std::pair<int, int>, double, PairHasher> edge_map;
        edge_map.clear();
        edge_map.reserve(dim * dim);

        // if (node->getBranchOnM() || node->getBranchOnN()) {
        //     edge_map[{node->getDim(), node->getDim()}] = 1.;
        //     return edge_map;
        // }

        auto &cols = node->cols;
        std::vector<double> x(cols.size());

        // SAFE_SOLVER(node->solver.getX(0, cols.size(), x.data()))
        SAFE_SOLVER(node->solver.getX(2, cols.size(), x.data()))
        std::vector<double> x0(1);
        SAFE_SOLVER(node->solver.getX(0, 1, x0.data()))
        x[0] = x0[0];
        for (int i = 0; i < cols.size(); ++i) {
            const auto val = x[i];
            if (val < SOL_X_TOLERANCE) continue;
            const auto &seq = cols[i].col_seq;
            int b4 = 0;
            for (auto j: seq) {
                if (b4 < j) edge_map[{b4, j}] += val;
                else edge_map[{j, b4}] += val;
                b4 = j;
            }
            if (seq.size() != 1) edge_map[{0, b4}] += val; //if single point, just be one count;
        }

        // print the edge_map
        global_config.ALL_EDGES_IF_ONE = true; //assume all edges are 1.
        // std::cout << "Edge map size: " << edge_map.size() << std::endl;
        for (const auto &pair : edge_map) {
            // std::cout << "Edge: " << pair.first.first << "-" << pair.first.second
            //           << ", Value: " << pair.second << std::endl;
            if (!equalFloat(pair.second, 1., EDGE_IF_ONE_TOLERANCE))
                global_config.ALL_EDGES_IF_ONE = false; //if one edge is not 1, then set the flag to false
        }
        // exit(0);    


        //check the edge
        for (auto &brc: node->brcs) {
            if ((brc.edge.first==node->getDim()) && (brc.edge.second==node->getDim())) continue; //skip the 0-0 edge
            if (brc.br_dir) {
                if (!equalFloat(edge_map[brc.edge], 1., EDGE_IF_ONE_TOLERANCE)) {
                    //print the col
                    for (int i = 0; i < cols.size(); ++i) {
                        const auto val = x[i];
                        if (val < SOL_X_TOLERANCE) continue;
                        const auto &seq = cols[i].col_seq;
                        if (std::find(seq.begin(), seq.end(), brc.edge.second) != seq.end()) {
                            std::cout << "col: " << i << " val: " << val << " seq: ";
                            for (auto j: seq) {
                                std::cout << j << " ";
                            }
                            std::cout << std::endl;
                        }
                    }

                    THROW_RUNTIME_ERROR(
                        "edge: " + std::to_string(brc.edge.first) + " " + std::to_string(brc.edge.second)
                        + " not 1 but " + std::to_string(edge_map[brc.edge]));
                } else {
                    edge_map[brc.edge] = 1.; //force to be 1.
                }
            } else {
                if (!equalFloat(edge_map[brc.edge], 0.)) {
                    THROW_RUNTIME_ERROR(
                        "edge: " + std::to_string(brc.edge.first) + " " + std::to_string(brc.edge.second)
                        + " not 0 but " + std::to_string(edge_map[brc.edge]));
                } else {
                    edge_map[brc.edge] = 0.; //force to be 0.
                }
            }
        }
        return edge_map;
    }
}

#endif // ROUTE_OPT_CALL_BRANCHING_HPP
