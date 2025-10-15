/* 
 * Copyright (c) 2025 Zhengzhong (Ricky) You.
 * All rights reserved.
 * Software: RouteOpt
 * License: GPL-3.0
 */

/*
 * @file candidate_selector_controller.hpp
 * @brief CandidateSelectorController class for managing candidate selection in RouteOpt.
 *
 * This header defines the BranchingTesting class, which encapsulates the candidate selection process
 * during branching in the RouteOpt framework. It manages different testing phases (LP, Heuristic, Exact)
 * by applying user-defined processing functions on nodes, and measuring the corresponding execution times.
 */

#ifndef ROUTE_OPT_CANDIDATE_SELECTOR_CONTROLLER_HPP
#define ROUTE_OPT_CANDIDATE_SELECTOR_CONTROLLER_HPP

#include <unordered_map>
#include <vector>
#include <functional>
#include <random>
#include "route_opt_macro.hpp"
#include "candidate_selector_macro.hpp"
#include "branching_macro.hpp"
#include "cvrp_macro.hpp"
#include "global_config.hpp"

namespace RouteOpt::Branching::CandidateSelector {
    /**
     * @brief The BranchingTesting class encapsulates the candidate selection process
     *        during branching in the RouteOpt framework.
     *
     * This templated class manages different testing phases (LP, Heuristic, Exact)
     * by applying user-defined processing functions on nodes, and measuring the
     * corresponding execution times. It also maintains phase-specific counters
     * and provides an interface to retrieve the best candidate branching decision.
     *
     * @tparam Node    Type representing a node in the branching tree.
     * @tparam BrCType Type representing a branching candidate.
     * @tparam Hasher  Hash function for candidate keys.
     */
    template<typename Node, typename BrCType, typename Hasher>
    class BranchingTesting {
    public:
        /**
         * @brief Constructs a BranchingTesting object with phase parameters and testing functions.
         *
         * @param num_phase0             Number of tests to perform in phase 0 (LP phase).
         * @param num_phase1             Number of tests to perform in phase 1 (Heuristic phase).
         * @param num_phase2             Number of tests to perform in phase 2 (Exact phase).
         * @param num_phase3             Number of tests to perform in phase 3.
         * @param processLPTestingFunction    Function to process LP testing.
         * @param processHeurTestingFunction  Function to process heuristic testing.
         * @param processExactTestingFunction Function to process exact testing.
         */
        BranchingTesting(int num_phase0, int num_phase1, int num_phase2, int num_phase3,
                         const std::function<void(Node *, const BrCType &, double &, double &)> &
                         processLPTestingFunction,
                         const std::function<void(Node *, const BrCType &, double &, double &)> &
                         processHeurTestingFunction,
                         const std::function<void(Node *, const BrCType &, double &, double &)> &
                         processExactTestingFunction) {
            setNumPhase0(num_phase0);
            setNumPhase1(num_phase1);
            setNumPhase2(num_phase2);
            setNumPhase3(num_phase3);
            setProcessLPTestingFunction(processLPTestingFunction);
            setProcessHeurTestingFunction(processHeurTestingFunction);
            setProcessExactTestingFunction(processExactTestingFunction);
        }

        /**
         * @brief Sets the number of tests for phase 0 (LP phase).
         * @param num Number of tests.
         */
        void setNumPhase0(int num) {
            num_phase0 = num;
        }

        /**
         * @brief Sets the number of tests for phase 1 (Heuristic phase).
         * @param num Number of tests.
         */
        void setNumPhase1(int num) {
            num_phase1 = num;
        }

        /**
         * @brief Sets the number of tests for phase 2 (Exact phase).
         * @param num Number of tests.
         */
        void setNumPhase2(int num) {
            num_phase2 = num;
        }

        /**
         * @brief Sets the number of tests for phase 3.
         * @param num Number of tests.
         */
        void setNumPhase3(int num) {
            num_phase3 = num;
        }

        /**
         * @brief Sets the function used for processing LP testing.
         * @param func Function to process LP testing.
         */
        void setProcessLPTestingFunction(const std::function<void(Node *, const BrCType &, double &, double &)> &func) {
            processLPTestingFunction = func;
        }

        /**
         * @brief Sets the function used for processing heuristic testing.
         * @param func Function to process heuristic testing.
         */
        void setProcessHeurTestingFunction(
            const std::function<void(Node *, const BrCType &, double &, double &)> &func) {
            processHeurTestingFunction = func;
        }

        /**
         * @brief Sets the function used for processing exact testing.
         * @param func Function to process exact testing.
         */
        void setProcessExactTestingFunction(
            const std::function<void(Node *, const BrCType &, double &, double &)> &func) {
            processExactTestingFunction = func;
        }

        /**
         * @brief Retrieves the edge score information generated during testing.
         * @return Constant reference to the vector of edge score information.
         */
        const std::vector<CandidateScoreInfo<BrCType> > &getEdgeInfo() const {
            return edge_info;
        }

        /**
         * @brief Gets the number of tests configured for phase 0.
         * @return Number of tests in phase 0.
         */
        [[nodiscard]] int getNumPhase0() const {
            return num_phase0;
        }

        /**
         * @brief Gets the number of tests configured for phase 1.
         * @return Number of tests in phase 1.
         */
        [[nodiscard]] int getNumPhase1() const {
            return num_phase1;
        }

        /**
         * @brief Gets the number of tests configured for phase 2.
         * @return Number of tests in phase 2.
         */
        [[nodiscard]] int getNumPhase2() const {
            return num_phase2;
        }

        /**
         * @brief Gets the number of tests configured for phase 3.
         * @return Number of tests in phase 3.
         */
        [[nodiscard]] int getNumPhase3() const {
            return num_phase3;
        }

        /**
         * @brief Provides a mutable reference to the LP testing time counter.
         * @return Reference to the LP time counter (pair: time and count).
         */
        auto &refLPTimeCnt() {
            return lp_time_cnt;
        }

        /**
         * @brief Provides a mutable reference to the heuristic testing time counter.
         * @return Reference to the heuristic time counter (pair: time and count).
         */
        auto &refHeuristicTimeCnt() {
            return heuristic_time_cnt;
        }

        /**
         * @brief Provides a mutable reference to the exact testing time counter.
         * @return Reference to the exact time counter (pair: time and count).
         */
        auto &refExactTimeCnt() {
            return exact_time_cnt;
        }

        /**
         * @brief Executes the testing phases and returns the best branching candidate.
         *
         * This function applies the LP, Heuristic, and Exact testing functions on the given node.
         * It measures the execution time for each phase and uses the branching history and shared
         * data to screen and select the best candidate.
         *
         * @param node                 Pointer to the current node.
         * @param branching_history    Reference to the branching history.
         * @param branching_data_shared Reference to the shared branching data.
         * @param candidate_map        Map of candidate branching decisions and their scores.
         * @return The best candidate branching decision.
         */
        const BrCType &getBestCandidate(Node *node,
                                        BranchingHistory<BrCType, Hasher> &branching_history,
                                        BranchingDataShared<BrCType, Hasher> &branching_data_shared,
                                        const std::unordered_map<BrCType, double, Hasher> &candidate_map) {

            static BrCType dummy_default;
            testing_range_branch(node);
            if (node->getBranchOnM() || node->getBranchOnN()) {
                // std::cout << "Branching on range: " << (node->getBranchOnM() ? "M" : "N") 
                //     << ", big value: " << (node->getBranchOnM() ? node->getBigU() : node->getBigL()) << std::endl;

                testing(node, branching_history, branching_data_shared, TestingPhase::LP);

                if (node->getBranchOnM() && node->getBranchOnN()) {
                    // generate random number to decide which one to branch on
                    // static std::random_device rd;
                    // static std::mt19937 gen(rd());
                    // std::uniform_real_distribution<double> dis(0.0, 1.0);
                    // double rnd = dis(gen);
                    // if (rnd < 0.5) {
                    //     std::cout << "Branching on both M and N, prioritize M branching." << std::endl;
                    //     node->setBranchOnN(false); // reset N branching for next iteration
                    //     std::cout << "Branching on M with big_U = " << node->getBigU() << std::endl;
                    // } else {
                    //     std::cout << "Branching on both M and N, prioritize N branching." << std::endl;
                    //     node->setBranchOnM(false); // reset N branching for next iteration
                    //     std::cout << "Branching on N with big_L = " << node->getBigL() << std::endl;
                    // }
                    int start = node->getDim();
                    std::vector<double> rhs_m_n(4);
                    SAFE_SOLVER(node->refSolver().getRhs(3*start-1+1, 4, rhs_m_n.data()))
                    double lb_m = rhs_m_n[0];
                    double ub_m = rhs_m_n[1];
                    double lb_n = rhs_m_n[2];
                    double ub_n = rhs_m_n[3];

                    double range_m = ub_m - lb_m;
                    double range_n = ub_n - lb_n;

                    // prioritize branching on the variable with larger range
                    if (range_m >= range_n) {
                        std::cout << "Branching on both M and N, prioritize M branching." << std::endl;
                        node->setBranchOnN(false); // reset N branching for next iteration
                        std::cout << "Branching on M with big_U = " << node->getBigU() << std::endl;
                    } else {
                        std::cout << "Branching on both M and N, prioritize N branching." << std::endl;
                        node->setBranchOnM(false); // reset N branching for next iteration
                        std::cout << "Branching on N with big_L = " << node->getBigL() << std::endl;
                    }

                }
                dummy_default.first = node->getDim();
                dummy_default.second = node->getDim();

                return dummy_default;
            }

            // exit(0);

            // here we are trying to branch on last customer
            // testing_last_customer_branch(node);
            // if (node->getBranchOnCustomer()) {

            //     testing(node, branching_history, branching_data_shared, TestingPhase::LP);

            //     if (node->getBranchOnCustomer()) {
            //         std::cout << "Branching on customer: " << node->getBranchCustomerIdx() << std::endl;
            //         dummy_default.first = node->getDim();
            //         dummy_default.second = node->getDim();
            //         return dummy_default;
            //     }       
            // }



            NoEdgeCandidate_LP = false;
            NoEdgeCandidate_Heuristic = false;
            NoEdgeCandidate_Exact = false;

            // Update the candidate map in the shared data.
            branching_data_shared.refCandidateMap() = candidate_map;
            // Perform initial screening based on the LP phase.
            branching_history.initialScreen(branching_data_shared, num_phase0);

            // Measure LP testing time.
            lp_time_cnt.first = TimeSetter::measure([&]() {
                testing(node, branching_history, branching_data_shared, TestingPhase::LP);
            });
            lp_time_cnt.second = num_phase0 == 1 ? 0 : 2 * num_phase0;
            // Measure heuristic testing time.
            heuristic_time_cnt.first = TimeSetter::measure([&]() {
                testing(node, branching_history, branching_data_shared, TestingPhase::Heuristic);
            });
            heuristic_time_cnt.second = num_phase1 == 1 ? 0 : 2 * num_phase1;
            // Measure exact testing time.
            exact_time_cnt.first = TimeSetter::measure([&]() {
                testing(node, branching_history, branching_data_shared, TestingPhase::Exact);
            });
            exact_time_cnt.second = num_phase2 == 1 ? 0 : 2 * num_phase2;

            if (NoEdgeCandidate_LP && NoEdgeCandidate_Heuristic && NoEdgeCandidate_Exact) {
                // dummy_default = {0, 0};
                // return dummy_default;
                return branching_data_shared.refBranchPair().front();
            }
            else {
                return branching_data_shared.refBranchPair().front();
            }   
        }

        /**
         * @brief Performs testing on the given node for a specified testing phase.
         *
         * This function is responsible for executing the appropriate testing function
         * (LP, Heuristic, or Exact) based on the provided phase parameter.
         *
         * @param node                 Pointer to the node being tested.
         * @param branching_history    Reference to the branching history.
         * @param branching_data_shared Reference to the shared branching data.
         * @param phase                The testing phase to execute.
         */
        void testing(
            Node *node,
            BranchingHistory<BrCType, Hasher> &branching_history,
            BranchingDataShared<BrCType, Hasher> &branching_data_shared,
            TestingPhase phase);

        void testing_range_branch(Node *node) {
            int num_col;
            SAFE_SOLVER(node->refSolver().getNumCol(&num_col))

            std::vector<double> xval(num_col);
            SAFE_SOLVER(node->refSolver().getX(0, num_col, xval.data()))


            bool branch_on_m = true;
            bool branch_on_n = true;

            int branch_var_idx_m = 1;
            int branch_var_idx_n = 2;

            double UB_L = xval[branch_var_idx_m];
            double LB_U = xval[branch_var_idx_n];

            // check if the UB_L and LB_U are integer
            if (std::abs(UB_L - std::round(UB_L)) <= TOLERANCE) branch_on_m = false;
            if (std::abs(LB_U - std::round(LB_U)) <= TOLERANCE) branch_on_n = false;

            std::vector<int> cbeg;
            std::vector<int> cind;
            std::vector<double> cval;
            int numnzP;
            int start = node->getDim();
            SAFE_SOLVER(node->refSolver().getConstraints(&numnzP, nullptr, nullptr, nullptr, start, 1))
            cbeg.resize(numnzP+1);
            cind.resize(numnzP);
            cval.resize(numnzP);
            SAFE_SOLVER(node->refSolver().getConstraints(&numnzP, cbeg.data(), cind.data(), cval.data(), start, 1))

            std::vector<double> rhs_m_n(4);
            SAFE_SOLVER(node->refSolver().getRhs(3*start-1+1, 4, rhs_m_n.data()))
            double lb_m = rhs_m_n[0];
            double ub_m = rhs_m_n[1];
            double lb_n = rhs_m_n[2];
            double ub_n = rhs_m_n[3];

            int dim = node->getDim();
            const auto &col = node->getCols();
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

            for (int i = 1; i < dim; ++i) {
                if (customer_x_contribution[i] > TOLERANCE)
                    customer_cost_contribution_n[i] += global_config.BIG_M;
            }


            // print customer contribution
            // std::cout << "Customer contribution: " << std::endl;
            // for (int i = 0; i < dim; ++i) {
            //     if (customer_x_contribution[i] > TOLERANCE)
            //         std::cout << "Customer " << i << ": " << customer_cost_contribution_m[i] << ", " 
            //             << customer_cost_contribution_n[i] << std::endl;
            // }

            // std::cout << "Customer x contribution: " << std::endl;
            // for (int i = 0; i < dim; ++i) {
            //     if (customer_x_contribution[i] > TOLERANCE)
            //         std::cout << "Customer " << i << ": " << customer_x_contribution[i] << std::endl;
            // }


            double max_m_bar = 0.0;
            double real_n_bar = std::numeric_limits<double>::infinity();
            double min_n_bar = std::numeric_limits<double>::infinity();
            for (int i = 1; i < dim; ++i) {
                if (customer_x_contribution[i] > TOLERANCE) {
                    max_m_bar = std::max(max_m_bar, customer_cost_contribution_m[i]);
                    real_n_bar = std::min(real_n_bar, customer_cost_contribution_m[i]);
                    min_n_bar = std::min(min_n_bar, customer_cost_contribution_n[i]);
                }
            }

            std::cout << "max_m_bar = " << max_m_bar << ", min_n_bar = " << min_n_bar << ", real_n_bar = " << real_n_bar << std::endl;


            if (!branch_on_m && !branch_on_n) {
                std::cout << "Both m and n are integer, no need to branch" << std::endl;
                // if ((max_m_bar < min_n_bar) && (max_m_bar > lb_m + TOLERANCE) && (max_m_bar < ub_m - TOLERANCE)) {
                //     node->setBranchOnM(true);
                //     node->setBranchOnN(false);
                //     node->setBigU(max_m_bar);
                //     std::cout << "set big_U = " << max_m_bar << std::endl;

                //     // node->setBranchOnM(false);
                //     // node->setBranchOnN(true);
                //     // node->setBigL(real_n_bar);
                //     // std::cout << "set big_L = " << real_n_bar << std::endl;
                //     return;
                // }
                node->setBranchOnM(branch_on_m);
                node->setBranchOnN(branch_on_n);

                return;
            }




            // // find the largest contribution in the customer_contribution vector
            // double max_contribution = 0.0;
            // int last_customer = -1;
            // for (int i = 1; i < dim; ++i) {
            //     if (equalFloat(customer_x_contribution[i], 1., TOLERANCE)) continue; // skip if the customer is not in the solution
            //     if (customer_cost_contribution_m[i] > max_contribution) {
            //         max_contribution = customer_cost_contribution_m[i];
            //         last_customer = i;
            //     }
            // }

            // std::cout << "last customer = " << last_customer << ", contribution = " << max_contribution << std::endl;




            double UB_U = -std::numeric_limits<double>::infinity();
            for (int i = 3; i < num_col; ++i) {
                if (xval[i] > TOLERANCE) {
                    UB_U = std::max(UB_U, cval[i-2]);
                }
            }


            double LB_L = std::numeric_limits<double>::infinity();
            for (int i = 3; i < num_col; ++i) {
                if (xval[i] > TOLERANCE) {
                    LB_L = std::min(LB_L, cval[i-2]);
                }
            }

            if (UB_U - UB_L <= TOLERANCE) branch_on_m = false;
            if (LB_U - LB_L <= TOLERANCE) branch_on_n = false;


            if (!branch_on_m && !branch_on_n) {
                std::cout << "Both m and n are range-respecting, no need to branch" << std::endl;
                node->setBranchOnM(branch_on_m);
                node->setBranchOnN(branch_on_n);
                return;
            }


            // double big_U = (1 + alpha) * UB_L;
            // double big_L = (1 - alpha) * LB_U;

            double big_U = UB_L;
            double big_L = LB_U;

            // if ((big_U - lb_m <= TOLERANCE) || (big_U - ub_m >= TOLERANCE)) branch_on_m = false;
            // if ((big_L - lb_n <= TOLERANCE) || (big_L - ub_n >= TOLERANCE)) branch_on_n = false;


            // if (!branch_on_m && !branch_on_n) {
            //     std::cout << "Both big U and big L are out of range, no need to branch" << std::endl;
            //     node->setBranchOnM(branch_on_m);
            //     node->setBranchOnN(branch_on_n);
            //     return;
            // }


            // if (branch_on_m && branch_on_n) {
            //     branch_on_m = false;
            // }

            
            node->setBigU(big_U);
            node->setBigL(big_L);

            node->setBranchOnM(branch_on_m);
            node->setBranchOnN(branch_on_n);

            // if (branch_on_m) std::cout << "branch_on_m = true, " << "big_U = " << big_U << std::endl;
            // if (branch_on_n) std::cout << "branch_on_n = true, " << "big_L = " << big_L << std::endl;

            node->setBranchOnCustomer(false);
            node->setBranchCustomerIdx(-1);
                

            // std::cout << "UB_L = " << UB_L << ", UB_U = " << UB_U << std::endl;
            // std::cout << "big_U = " << big_U << std::endl;


            // std::cout << "LB_U = " << LB_U << ", LB_L = " << LB_L << std::endl;
            // std::cout << "big_L = " << big_L << std::endl;
        }

        
        
        void testing_last_customer_branch(Node *node) {

            // 1.1 check which last customer contribute to the largest route cost
            // 1.2 check if the last customer is fractional
            // 1.3 if yes, branch on it and return
               // 1.3.1 branch into two children nodes, one with use this last customer as one of route, the other one without
               // 1.3.2 add constriants to the two children nodes, one with x_{i,last_customer} = 1, the other one with x_{i,last_customer} = 0
        
            int num_col;
            SAFE_SOLVER(node->refSolver().getNumCol(&num_col))
            std::vector<double> xval(num_col);
            SAFE_SOLVER(node->refSolver().getX(0, num_col, xval.data()))

            int dim = node->getDim();
            const auto &col = node->getCols();

            std::vector<int> cbeg;
            std::vector<int> cind;
            std::vector<double> cval;
            int numnzP;
            SAFE_SOLVER(node->refSolver().getConstraints(&numnzP, nullptr, nullptr, nullptr, dim, 1))
            cbeg.resize(numnzP+1);
            cind.resize(numnzP);
            cval.resize(numnzP);
            SAFE_SOLVER(node->refSolver().getConstraints(&numnzP, cbeg.data(), cind.data(), cval.data(), dim, 1))

            std::vector<double> customer_cost_contribution(dim, 0.0);
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
                        customer_cost_contribution[last_customer] += cost * xval[i];
                        customer_cost_contribution_n[last_customer] += (cost - global_config.BIG_M) * xval[i];
                        customer_x_contribution[last_customer] += xval[i];
                    }
                }
            }


            // print customer contribution
            std::cout << "Customer contribution: " << std::endl;
            for (int i = 0; i < dim; ++i) {
                if (customer_cost_contribution[i] > TOLERANCE)
                    std::cout << "Customer " << i << ": " << customer_cost_contribution[i] << ", " 
                        << customer_cost_contribution_n[i] + global_config.BIG_M << std::endl;
            }

            std::cout << "Customer x contribution: " << std::endl;
            for (int i = 0; i < dim; ++i) {
                if (customer_x_contribution[i] > TOLERANCE)
                    std::cout << "Customer " << i << ": " << customer_x_contribution[i] << std::endl;
            }



            // find the largest contribution in the customer_contribution vector
            double max_contribution = 0.0;
            int last_customer = -1;
            for (int i = 1; i < dim; ++i) {
                if (equalFloat(customer_x_contribution[i], 1., TOLERANCE)) continue; // skip if the customer is not in the solution
                if (customer_cost_contribution[i] > max_contribution) {
                    max_contribution = customer_cost_contribution[i];
                    last_customer = i;
                }
            }

            // std::cout << "last customer = " << last_customer << ", contribution = " << max_contribution << std::endl;

            // find the last brc
            
            if (last_customer == -1) {
                std::cout << "last customer is already branched on, skip" << std::endl;
                node->setBranchOnCustomer(false);
                node->setBranchCustomerIdx(-1);
                return;
            }

            node->setBranchOnCustomer(true);
            node->setBranchCustomerIdx(last_customer);

            node->setBranchOnM(false);
            node->setBranchOnN(false);
        
        } 

        /**
         * @brief Updates the BKF controllers with the measured testing times.
         *
         * This function iterates over the provided BKF controllers and sets their testing
         * and node processing times based on the measured time counters from different testing phases.
         *
         * @param eps             Additional elapsed time.
         * @param bkf_controllers Vector of BKFController objects to update.
         */
        void updateBKFtime(double eps, std::vector<BKF::BKFController> &bkf_controllers) const {
            for (int i = 0; i < bkf_controllers.size(); ++i) {
                auto &bkf = bkf_controllers[i];
                if (i == 0) {
                    bkf.setTestingTime(lp_time_cnt.first, lp_time_cnt.second);
                    bkf.setNodeTime(eps + (heuristic_time_cnt.first + exact_time_cnt.first) / 2);
                } else if (i == 1) {
                    bkf.setTestingTime(heuristic_time_cnt.first, heuristic_time_cnt.second);
                    bkf.setNodeTime(eps + (exact_time_cnt.first + lp_time_cnt.first) / 2);
                } else if (i == 2) {
                    bkf.setTestingTime(exact_time_cnt.first, exact_time_cnt.second);
                    bkf.setNodeTime(eps + (lp_time_cnt.first + heuristic_time_cnt.first) / 2);
                } else
                    THROW_RUNTIME_ERROR("BKFController only supports 3 phases, but got " + std::to_string(i));
            }
        }

        // Delete the default constructor to enforce proper initialization.
        BranchingTesting() = delete;

        ~BranchingTesting() = default;

    private:
        // Number of testing phases for LP, Heuristic, Exact, and an additional phase.
        int num_phase0{};
        int num_phase1{};
        int num_phase2{};
        int num_phase3{};
        // Time counters for each testing phase (pair: total time, count of tests).
        std::pair<double, int> lp_time_cnt{}; // LP testing time and count.
        std::pair<double, int> heuristic_time_cnt{}; // Heuristic testing time and count.
        std::pair<double, int> exact_time_cnt{}; // Exact testing time and count.
        // Edge score information collected during testing.
        std::vector<CandidateScoreInfo<BrCType> > edge_info{};
        // Function objects for processing each type of testing.
        std::function<void(Node *, const BrCType &, double &, double &)> processLPTestingFunction{};
        std::function<void(Node *, const BrCType &, double &, double &)> processHeurTestingFunction{};
        std::function<void(Node *, const BrCType &, double &, double &)> processExactTestingFunction{};

        /**
         * @brief Revises the score for extremely unbalanced candidates.
         *
         * This function adjusts the candidate scores for a
         * given testing phase if the scores are extremely unbalanced.
         *
         * @param branching_history Reference to the branching history.
         * @param phase             The testing phase for which to revise scores.
         */
        void reviseExtremeUnbalancedScore(BranchingHistory<BrCType, Hasher> &branching_history,
                                          TestingPhase phase);
    };
}

#include "candidate_testing.hpp"
#include "initial_screen.hpp"
#endif // ROUTE_OPT_CANDIDATE_SELECTOR_CONTROLLER_HPP
