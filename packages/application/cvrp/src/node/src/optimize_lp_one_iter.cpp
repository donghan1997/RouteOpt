/* 
 * Copyright (c) 2025 Zhengzhong (Ricky) You.
 * All rights reserved.
 * Software: RouteOpt
 * License: GPL-3.0
 */

#include "node.hpp"

namespace RouteOpt::Application::CVRP {
    void BbNode::optimizeLPForOneIteration(double &prior_value, bool if_allow_delete_col, int lp_method) {
        int num_col;

        // SAFE_SOLVER(solver.setEnvOutputFlag(1, 0))
        SAFE_SOLVER(solver.reoptimize(lp_method))
        SAFE_SOLVER(solver.getNumCol(&num_col))
        // std::cout << "Number of columns in the node: " << num_col << std::endl;

        double val;
        SAFE_SOLVER(solver.getObjVal(&val))
        // std::cout << "LP value: " << val << std::endl;

        // std::vector<double> xval(num_col);
        // SAFE_SOLVER(solver.getX(0, num_col, xval.data()))
        // std::cout << "col 0 values: " << xval[0] << std::endl;
        // std::cout << "m values: " << xval[1] << std::endl;
        // std::cout << "n values: " << xval[2] << std::endl;
        

        if (!if_in_enu_state && if_allow_delete_col && num_col > LP_COL_FINAL_LIMIT) {
            double tol = std::max(TOLERANCE * val, TOLERANCE);
            if (std::abs(prior_value - val) > tol) {
                cleanIndexColForNode();
            }
        }
        prior_value = val;
    }
}
