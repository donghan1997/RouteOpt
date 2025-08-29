/* 
 * Copyright (c) 2025 Zhengzhong (Ricky) You.
 * All rights reserved.
 * Software: RouteOpt
 * License: GPL-3.0
 */

#include "cvrp.hpp"
#include "cvrp_macro.hpp"

namespace RouteOpt::Application::CVRP {
    void CVRPSolver::callPricingAtBeg(BbNode *node) {

        int num_col;
        SAFE_SOLVER(node->refSolver().getNumCol(&num_col))
        
        setEnv(node);
        if constexpr (ml_type == ML_TYPE::ML_GET_DATA_1 || ml_type == ML_TYPE::ML_GET_DATA_2) {
            GetTrainingData<BbNode, std::pair<int, int>,
                PairHasher>::checkIfStopGeneratingData(node->getTreeSize(), node->refIfTerminate());
            if (node->getIfTerminate()) return;
        }
        
        SAFE_SOLVER(node->refSolver().getNumCol(&num_col))
        // std::cout << "Before callPricing, Number of columns in the node: " << num_col << std::endl;
        // exit(0);
        double eps;
        // std::cout << "call pricing at beg, node idx = " << node->getIdx() << std::endl;
        callPricing(node, std::numeric_limits<float>::max(), eps);

        // std::cout << "Finish callPricing at beg, node idx = " << node->getIdx() << std::endl;
        if (node->getIfRootNode() && app_type == APPLICATION_TYPE::VRPTW)
            augmentNGRound(
                node, pricing_controller.refNG());
        
        // node->refIfTerminate() = true;       
    }

    void CVRPSolver::callPricing(BbNode *node, double labeling_time_limit, double &time_4_pure_pricing) {
        if (node->getIfInEnumState()) {
        ENU:
            callInspection(node, time_4_pure_pricing);
            BbNode::regenerateEnumMat(node, nullptr, false, optimal_dual_vector);
        } else {
            // std::cout << "call labeling at node idx = " << node->getIdx() << std::endl;
            callLabeling(node, labeling_time_limit, time_4_pure_pricing);
            // std::cout << "Finish labeling at node idx = " << node->getIdx() << std::endl;
            if (node->getIfInEnumState()) {
                goto ENU;
            }
            if constexpr (ml_type != ML_TYPE::ML_GET_DATA_1 || ml_type != ML_TYPE::ML_GET_DATA_2) {
                if (pricing_controller.getIfCompleteCG()) {
                    std::vector<int> cstr_index;
                    node->findNonActiveCuts(optimal_dual_vector, cstr_index);
                    SAFE_SOLVER(node->refSolver().reoptimize(SOLVER_DUAL_SIMPLEX));
                }
            }
        }

        // std::cout << "here is the node idx = " << node->getIdx() << std::endl;

        if (glob_timer.getTime() > TIME_LIMIT) {
            PRINT_REMIND("time limit reached!");
            node->refIfTerminate() = true;
        }

        if constexpr (ml_type != ML_TYPE::ML_NO_USE) {
            if (!node->getIfTerminate()) l2b_controller.recordEdgeLongInfo(BbNode::obtainSolEdgeMap(node));
        }
    }

    void CVRPSolver::callInspection(BbNode *node, double &time_4_pure_pricing) {
        constexpr bool if_update_column_pool = true;
        constexpr bool if_allow_delete_col = true;
        time_4_pure_pricing = TimeSetter::measure([&]() {
            solveLPByInspection(node, if_update_column_pool, if_allow_delete_col);
        });
    };

    void CVRPSolver::callLabeling(BbNode *node, double labeling_time_limit, double &time_4_pure_pricing) {
        constexpr bool if_open_heur = true;
        constexpr bool if_open_exact = true;
        constexpr bool if_update_node_val = true;
        constexpr bool if_possible_terminate_early = false;
        constexpr bool if_fix_row = false;
        constexpr bool if_allow_delete_col = false;

        bool if_fix_meet_point = !node->getIfRootNode();
        bool if_consider_regenerate_bucket_graph = node->getIfRootNode();

        int num_col;
        SAFE_SOLVER(node->refSolver().getNumCol(&num_col))
        // std::cout << "Before solveLPInLabeling, Number of columns in the node: " << num_col << std::endl;

        // std::cout << "Start solveLPInLabeling at node idx = " << node->getIdx() << std::endl;
        time_4_pure_pricing = TimeSetter::measure([&]() {
            solveLPInLabeling(node, if_open_heur, if_open_exact, if_update_node_val,
                              if_consider_regenerate_bucket_graph, if_possible_terminate_early,
                              if_fix_row, if_fix_meet_point, if_allow_delete_col, labeling_time_limit);
        });
        // std::cout << "Finish solveLPInLabeling at node idx = " << node->getIdx() << std::endl;

        SAFE_SOLVER(node->refSolver().getNumCol(&num_col))
        // std::cout << "After solveLPInLabeling, Number of columns in the node: " << num_col << std::endl;
        std::vector<double> opt_sol(num_col);
        SAFE_SOLVER(node->refSolver().getX(0, num_col, opt_sol.data()))
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

        // get cols size
        std::cout << "size of cols = " << num_col << std::endl;
        std::cout << "route size = " << col.size() << std::endl;
        std::cout << "m = " << opt_sol[1] << " , n = " << opt_sol[2] << std::endl;
        for (int i = 3; i < num_col; ++i) {
            if (opt_sol[i] > TOLERANCE) {
                std::cout << "Column " << i << ": ";
                auto &ci = col[i-2];
                auto &seq = ci.col_seq;                
                for (const auto &s: seq) {

                    std::cout << s << " ";
                }
                std::cout << ", x = : " << opt_sol[i] << " , cost = " << cval[i-2] << std::endl;
            }
        }
        // exit(0);




        if (node->getIfTerminate() || !pricing_controller.getIfCompleteCG()) return;
        if (!node->getIfRootNode() && (ml_type == ML_TYPE::ML_GET_DATA_1 || ml_type == ML_TYPE::ML_GET_DATA_2)) return;
        pricing_controller.eliminateArcs<!IF_SYMMETRY_PROHIBIT>(node->getRCCs(), node->getR1Cs(),
                                                                node->getBrCs(), optimal_dual_vector, ub,
                                                                node->calculateOptimalGap(ub),
                                                                node->refLastGap(),
                                                                node->refNumForwardBucketArcs(),
                                                                node->refNumBackwardBucketArcs(),
                                                                node->refNumForwardJumpArcs(),
                                                                node->refNumBackwardJumpArcs());

        if constexpr (ml_type == ML_TYPE::ML_GET_DATA_1 || ml_type == ML_TYPE::ML_GET_DATA_2) return;
        if (!node->getRCCs().empty() || !node->getR1Cs().empty()) callEnumeration(node);
    };
}
