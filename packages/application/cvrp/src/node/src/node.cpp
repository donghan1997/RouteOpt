/* 
 * Copyright (c) 2025 Zhengzhong (Ricky) You.
 * All rights reserved.
 * Software: RouteOpt
 * License: GPL-3.0
 */

#include "node.hpp"

#include "node_macro.hpp"
#include "cvrp_macro.hpp"
#include "global_config.hpp"

namespace RouteOpt::Application::CVRP {
    int BbNode::dim{};
    int BbNode::node_idx_counter{};
    sparseRowMatrixXd BbNode::row_basic_matrix{};

    void BbNode::buildModel(int num_vehicle, int dim, Solver *solver, BbNode *node) {

        std::cout << "Start Building fair node model " <<  std::endl;

        // std::cout << "Before, BIG_M = " << BIG_M << ", Budget = " << Budget << std::endl;

        // BIG_M = global_config.BIG_M;
        // Budget = global_config.Budget;

        // std::cout << "After, BIG_M = " << BIG_M << ", Budget = " << Budget << std::endl;
 
        BbNode::dim = dim;
        node->if_root_node = true;

        int num_rows = dim - 1 + 1 + 1 + 2 * (dim-1) + 4 + 1;
        int num_cols = 3; // m and n and x


        std::vector<int> solver_beg, solver_ind;
        std::vector<double> solver_val, solver_obj;

        solver_obj.emplace_back(OBJ_ARTIFICIAL); // x
        // solver_obj.emplace_back(0.0); // x
        solver_obj.emplace_back(1.0); // m
        solver_obj.emplace_back(-1.0); // n

        node->solver.getEnv(solver);
        SAFE_SOLVER(node->solver.newModel(MODEL_NAME, 0, nullptr, nullptr, nullptr, nullptr, nullptr))

        SAFE_SOLVER(node->solver.addVars(num_cols,
            0,
            nullptr,
            nullptr,
            nullptr,
            solver_obj.data(),
            nullptr,
            nullptr,
            nullptr,
            nullptr))

        std::vector<double> solver_rhs;
        std::vector<char> solver_sense; 

        node->cols.emplace_back();
        auto &col = node->cols.back();

        // int numnz = 0;
        // int col_idx = 0;
        solver_beg.emplace_back(0); // start of the first column
        for (int i = 1; i < dim; ++i) {
            col.col_seq.emplace_back(i);

            solver_ind.emplace_back(0);
            solver_val.emplace_back(1.0);

            solver_beg.emplace_back(static_cast<int>(solver_ind.size()));

            solver_rhs.emplace_back(1.0);
            solver_sense.emplace_back(SOLVER_EQUAL);
        }


        solver_ind.emplace_back(0);
        solver_val.emplace_back(num_vehicle);
        solver_beg.emplace_back(static_cast<int>(solver_ind.size()));
        solver_rhs.emplace_back(num_vehicle);
        solver_sense.emplace_back(SOLVER_EQUAL);


        solver_ind.emplace_back(0);
        solver_val.emplace_back(global_config.Budget);
        solver_beg.emplace_back(static_cast<int>(solver_ind.size()));
        solver_rhs.emplace_back(global_config.Budget); // budget
        solver_sense.emplace_back(SOLVER_LESS_EQUAL);


        int last_customer = dim - 1;
        for (int i = 1; i < dim; ++i) {
            // tips: solver_ind.size() replace numnz
            solver_ind.emplace_back(1);
            solver_val.emplace_back(-1.0);
            solver_beg.emplace_back(static_cast<int>(solver_ind.size()));

            solver_rhs.emplace_back(0.0);
            solver_sense.emplace_back(SOLVER_LESS_EQUAL);
        }

        for (int i = 1; i < dim; ++i) {

            // solver_ind.emplace_back(0);
            // solver_val.emplace_back(-BIG_M);
            solver_ind.emplace_back(2);
            solver_val.emplace_back(-1.0);
            solver_beg.emplace_back(static_cast<int>(solver_ind.size()));

            solver_rhs.emplace_back(-global_config.BIG_M);
            solver_sense.emplace_back(SOLVER_GREATER_EQUAL);
        }

        solver_ind.emplace_back(1);
        solver_val.emplace_back(1.0);
        solver_ind.emplace_back(2);
        solver_val.emplace_back(-1.0);
        solver_beg.emplace_back(static_cast<int>(solver_ind.size()));
        solver_rhs.emplace_back(0.0);
        solver_sense.emplace_back(SOLVER_GREATER_EQUAL);

        solver_ind.emplace_back(1);
        solver_val.emplace_back(1.0);
        solver_beg.emplace_back(static_cast<int>(solver_ind.size()));
        solver_rhs.emplace_back(0.0);
        solver_sense.emplace_back(SOLVER_GREATER_EQUAL);

        solver_ind.emplace_back(1);
        solver_val.emplace_back(1.0);
        solver_beg.emplace_back(static_cast<int>(solver_ind.size()));
        solver_rhs.emplace_back(global_config.BIG_M);
        // solver_rhs.emplace_back(3000.0);
        solver_sense.emplace_back(SOLVER_LESS_EQUAL);

        solver_ind.emplace_back(2);
        solver_val.emplace_back(1.0);
        solver_beg.emplace_back(static_cast<int>(solver_ind.size()));
        solver_rhs.emplace_back(0.0);
        solver_sense.emplace_back(SOLVER_GREATER_EQUAL);

        solver_ind.emplace_back(2);
        solver_val.emplace_back(1.0);
        solver_beg.emplace_back(static_cast<int>(solver_ind.size()));
        solver_rhs.emplace_back(global_config.BIG_M);
        // solver_rhs.emplace_back(3000.0);
        solver_sense.emplace_back(SOLVER_LESS_EQUAL);







        col.forward_concatenate_pos = static_cast<int>(col.col_seq.size()) - 1;

        
        SAFE_SOLVER(node->solver.addConstraints(
                num_rows,
                static_cast<int>(solver_ind.size()),
                solver_beg.data(),
                solver_ind.data(),
                solver_val.data(),
                solver_sense.data(),
                solver_rhs.data(),
                nullptr)
        )

        SAFE_SOLVER(node->solver.updateModel())

        int num_col;
        SAFE_SOLVER(node->refSolver().getNumCol(&num_col))
        std::cout << "Number of cols in the node: " << num_col << std::endl;

        std::cout << "Finish  building fair node model " <<  std::endl;

        node->solver.optimize();

        // exit(0);

        // BbNode::dim = dim;
        // node->if_root_node = true;
        // std::vector<int> solver_beg, solver_ind;
        // std::vector<double> solver_val, solver_obj;

        // node->cols.emplace_back();
        // solver_beg.emplace_back(0);
        // auto &col = node->cols.back();
        // for (int i = 1; i < dim; ++i) {
        //     col.col_seq.emplace_back(i);
        //     solver_ind.emplace_back(i - 1);
        //     solver_val.emplace_back(1);
        // }
        // solver_ind.emplace_back(dim - 1);
        // solver_val.emplace_back(num_vehicle);
        // solver_beg.emplace_back(static_cast<int>(solver_ind.size()));
        // col.forward_concatenate_pos = static_cast<int>(col.col_seq.size()) - 1;

        // solver_obj.emplace_back(OBJ_ARTIFICIAL);
        // std::vector<double> rhs(dim, 1);
        // std::vector<char> sense(dim, SOLVER_EQUAL);

        // rhs[dim - 1] = num_vehicle;
        // sense[dim - 1] = SOLVER_GREATER_EQUAL;

        // node->solver.getEnv(solver);

        // SAFE_SOLVER(node->solver.newModel(MODEL_NAME, 0, nullptr, nullptr, nullptr, nullptr, nullptr))
        // SAFE_SOLVER(node->solver.addConstraints(
        //         dim,
        //         0,
        //         nullptr,
        //         nullptr,
        //         nullptr,
        //         sense.data(),
        //         rhs.data(),
        //         nullptr)
        // )
        // SAFE_SOLVER(node->solver.addVars(1,
        //     static_cast<int>(solver_ind.size()),
        //     solver_beg.data(),
        //     solver_ind.data(),
        //     solver_val.data(),
        //     solver_obj.data(),
        //     nullptr,
        //     nullptr,
        //     nullptr,
        //     nullptr))
        // SAFE_SOLVER(node->solver.updateModel())
    }


    void BbNode::createBasicMatrix() {
        auto size_enumeration_col_pool = static_cast<int>(index_columns_in_enumeration_column_pool.size());
        if (size_enumeration_col_pool > 0) {
            basic_matrix = (matrix_in_enumeration.front()).block(
                0, 0, dim - 1, size_enumeration_col_pool);
        } else {
            basic_matrix = Eigen::SparseMatrix<double>(dim - 1, 0);
        }
    }


    BbNode::~BbNode() {
        if (all_forward_buckets) {
            for (int i = 0; i < dim; ++i) {
                delete[]all_forward_buckets[i];
            }
            delete[]all_forward_buckets;
        }

        if (all_backward_buckets) {
            for (int i = 0; i < dim; ++i) {
                delete[]all_backward_buckets[i];
            }
            delete[]all_backward_buckets;
        }
        solver.freeModel();
    }
}
