/* 
 * Copyright (c) 2025 Zhengzhong (Ricky) You.
 * All rights reserved.
 * Software: RouteOpt
 * License: GPL-3.0
 */

#include <numeric>
#include "cvrp.hpp"
#include "cvrp_macro.hpp"
#include "candidate_selector_macro.hpp"

namespace RouteOpt::Application::CVRP {
    namespace ImposeBranchingDetail {
        inline void deleteArcByFalseBranchConstraint(int num_buckets_per_vertex, Bucket **buckets,
                                                     const std::pair<int, int> &edge) {
            int i = edge.first, j = edge.second;
            bool if_rep = false;
        AGAIN:
            for (int bin = 0; bin < num_buckets_per_vertex; ++bin) {
                auto if_find = std::find(buckets[i][bin].bucket_arcs.begin(),
                                         buckets[i][bin].bucket_arcs.end(), j);
                if (if_find == buckets[i][bin].bucket_arcs.end()) {
                    auto iff = std::find_if(buckets[i][bin].jump_arcs.begin(),
                                            buckets[i][bin].jump_arcs.end(),
                                            [&](const std::pair<res_int, int> &p) { return p.second == j; });
                    if (iff != buckets[i][bin].jump_arcs.end()) {
                        buckets[i][bin].jump_arcs.erase(iff);
                    }
                } else {
                    buckets[i][bin].bucket_arcs.erase(if_find);
                }
            }
            if (!if_rep) {
                if_rep = true;
                std::swap(i, j);
                goto AGAIN;
            }
        }

        void addBranchCutToUnsolved(
            BbNode *node,
            BbNode *&node2,
            const std::pair<int, int> &info,
            int num_buckets_per_vertex,
            bool if_symmetry
        ) {
            if (node->getIfTerminate()) return;

            int num_row;
            SAFE_SOLVER(node->refSolver().getNumRow(&num_row))
            Brc bf;
            bf.edge = info;
            // std::cout << "branch on edge: " << info.first << " " << info.second << std::endl;
            bf.idx_brc = num_row;
            std::vector<int> solver_ind;
            std::vector<double> solver_val;
            node->obtainBrcCoefficient(info, solver_ind, solver_val);
            // print the branch constraint
            // std::cout << "branch constraint: ";
            // for (const auto &i: solver_ind) {
            //     std::cout << i << " ";
            // }
            // std::cout << std::endl;
            // for (const auto &i: solver_val) {
            //     std::cout << i << " ";
            // }
            // std::cout << std::endl;

            bf.br_dir = true;
            node2 = new BbNode(node, if_symmetry, num_buckets_per_vertex, bf);
            TestingDetail::addBranchConstraint(solver_ind, solver_val, node2->refSolver(), true);

            bf.br_dir = false;
            bf.idx_brc = INVALID_BRC_INDEX;
            node->refBrCs().emplace_back(bf);
            if (solver_ind.front() == 0) {
                solver_ind.erase(solver_ind.begin());
            }
            node->rmLPCols(solver_ind);

            deleteArcByFalseBranchConstraint(num_buckets_per_vertex, node->refAllForwardBuckets(),
                                             info);
            if (!if_symmetry) {
                deleteArcByFalseBranchConstraint(num_buckets_per_vertex,
                                                 node->refAllBackwardBuckets(),
                                                 info);
            }
            node->getNewIdx();
        }

        // apply range branch for m and n
        void addRangeBranchCutToUnsolved(BbNode *node,
            BbNode *&node2,
            const std::pair<int, int> &info,
            int num_buckets_per_vertex,
            bool if_symmetry
        ) {

            if (node->getIfTerminate()) return;

            int num_col;
            SAFE_SOLVER(node->refSolver().getNumCol(&num_col))

            // std::vector<double> xval(num_col);
            // SAFE_SOLVER(node->refSolver().getX(0, num_col, xval.data()))

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


            int num_row;
            SAFE_SOLVER(node->refSolver().getNumRow(&num_row))

            Brc bf;
            bf.edge = info;
            bf.br_dir = true;
            bf.idx_brc = num_row;
            if (node->getBranchOnM()) {
                bf.range.first = 1;
                bf.range.second = static_cast<int>(node->getBigU());
            }
            if (node->getBranchOnN()) {

                bf.range.first = 2;
                bf.range.second = static_cast<int>(std::ceil(node->getBigL()));
            }         
            node2 = new BbNode(node, if_symmetry, num_buckets_per_vertex, bf);
            std::vector<int> solver_ind;
            std::vector<double> solver_val;
            if (node->getBranchOnM()) {

                double upper_m = node->getBigU();
                double lower_m = std::ceil(upper_m);
                std::cout << "branching on m" <<  std::endl;
                std::cout << "m <= " << static_cast<int>(upper_m) << std::endl;

               

                TestingDetail::addRangeBranchConstraint(3*start+1, static_cast<int>(upper_m), node2->refSolver());
                for (int i = 3; i < num_col; ++i) {
                    if (cval[i-2] - static_cast<int>(upper_m) >= TOLERANCE) {
                        solver_ind.emplace_back(i);
                        solver_val.emplace_back(1.0);
                    }
                }
                TestingDetail::addRangeConstraint(solver_ind, solver_val, node2->refSolver());
                
                
                bf.br_dir = false;
                bf.idx_brc = INVALID_BRC_INDEX;
                node->refBrCs().emplace_back(bf);
                TestingDetail::addRangeBranchConstraint(3*start-1+1, static_cast<int>(upper_m)+1, node->refSolver());
                std::cout << "m >= " << static_cast<int>(upper_m)+1 << std::endl;

            }
            else if (node->getBranchOnN()) {

                
                std::cout << "branch on n" << std::endl;

                double upper_n = node->getBigL();
                double lower_n = std::ceil(upper_n);


                TestingDetail::addRangeBranchConstraint(3*start+1+1, static_cast<int>(lower_n), node2->refSolver());
                for (int i = 3; i < num_col; ++i) {
                    if (cval[i-2] - static_cast<int>(lower_n) <= TOLERANCE) {
                        solver_ind.emplace_back(i);
                        solver_val.emplace_back(1.0);
                    }
                }
                TestingDetail::addRangeConstraint(solver_ind, solver_val, node2->refSolver());
                std::cout << "n >= " << static_cast<int>(lower_n) << std::endl;


                bf.br_dir = false;
                bf.idx_brc = INVALID_BRC_INDEX;
                node->refBrCs().emplace_back(bf);
                
                
                TestingDetail::addRangeBranchConstraint(3*start+2+1,static_cast<int>(lower_n)-1 , node->refSolver());
                std::cout << "n <= " << static_cast<int>(lower_n)-1 << std::endl;
                
            }
            else {
                std::cout << "no branching on m and n" << std::endl;
                return;
            }
            
            
            node->getNewIdx();


        }

        void addCustomerBranchCutToUnsolved(BbNode *node,
            BbNode *&node2,
            const std::pair<int, int> &info,
            int num_buckets_per_vertex,
            bool if_symmetry
        ) {

            if (node->getIfTerminate()) return;


            int num_row;
            SAFE_SOLVER(node->refSolver().getNumRow(&num_row))
            int branch_customer_idx = node->getBranchCustomerIdx();

            // std::cout << "branch on customer: " << branch_customer_idx << std::endl;

            Brc bf;
            bf.edge = info;
            bf.br_dir = true;
            bf.idx_brc = num_row;
            bf.last_customer_idx = branch_customer_idx;  
            node2 = new BbNode(node, if_symmetry, num_buckets_per_vertex, bf);
            std::vector<int> solver_ind;
            std::vector<double> solver_val;

            
            auto &cols = node->getCols();
            for (int i = 1; i < cols.size(); ++i) {
                if (cols[i].col_seq.back() == branch_customer_idx) {
                    // print cols[i].col_seq
                    // std::cout << "cols[" << i+2 << "].col_seq: ";
                    // for (const auto &j: cols[i].col_seq) {
                    //     std::cout << j << " ";
                    // }
                    // std::cout << std::endl;
                    solver_ind.emplace_back(i + 2);
                    solver_val.emplace_back(1.0);
                }
            }
            TestingDetail::addCustomerBranchConstraint(solver_ind, solver_val, node2->refSolver(), true);
                      

            bf.br_dir = false;
            bf.idx_brc = num_row;
            // bf.last_customer_idx = branch_customer_idx;
            node->refBrCs().emplace_back(bf);
            TestingDetail::addCustomerBranchConstraint(solver_ind, solver_val, node->refSolver(), false);
            node->getNewIdx();

        }


        void addBranchCutToUnsolvedInEnu(BbNode *node, BbNode *&node2, const std::pair<int, int> &info,
                                         int *col_pool4_pricing) {
            if (node->getIfTerminate()) return;

            std::vector<int> ind_use;
            std::vector<double> solver_val;
            node->obtainBrcCoefficient(info, ind_use, solver_val);
            if (ind_use.front() == 0) {
                ind_use.erase(ind_use.begin());
            }

            std::vector<int> ind_not_allow;
            node->obtainColIdxNotAllowByEdge(info, ind_not_allow);
            if (ind_not_allow.front() == 0) {
                ind_not_allow.erase(ind_not_allow.begin());
            }

            int num_row;
            SAFE_SOLVER(node->refSolver().getNumRow(&num_row))
            std::vector<double> duals(num_row, -1); //to prevent any constraints are deleted!

            Brc bf;
            bf.edge = info;
            bf.br_dir = true;
            bf.idx_brc = INVALID_BRC_INDEX;
            node2 = new BbNode(node, bf);
            node2->rmLPCols(ind_not_allow);
            if (node->refIndexColumnsInEnumerationColumnPool().size() != 0) {
                node->rmColByBranchInEnuMatrix(node2->refDeletedColumnsInEnumerationColumnPool(), true, {bf},
                                               col_pool4_pricing);
                BbNode::regenerateEnumMat(node, node2, false, duals);
            }

            bf.br_dir = false;
            node->refBrCs().emplace_back(bf);
            node->rmLPCols(ind_use);
            if (node->refIndexColumnsInEnumerationColumnPool().size() != 0) {
                node->rmColByBranchInEnuMatrix(node->refDeletedColumnsInEnumerationColumnPool(), true, {bf},
                                               col_pool4_pricing);
                BbNode::regenerateEnumMat(node, node, false, duals);
            }
            node->getNewIdx();
        }
    }

    void CVRPSolver::imposeBranching(BbNode *node, const std::pair<int, int> &brc, std::vector<BbNode *> &children) {
        children.resize(2);
        children.front() = node;

        // std::cout << "impose branching" << std::endl;

        if (node->getIfInEnumState()) {
            ImposeBranchingDetail::addBranchCutToUnsolvedInEnu(
                node, children.back(), brc, pricing_controller.getColumnPoolPtr());
        } else {
            // std::cout << "addBranchCutToUnsolved" << std::endl;
            // apply range branch
            if ((brc.first==node->getDim()) && (brc.second==node->getDim())) {

                if (node->getBranchOnM() || node->getBranchOnN()) {
                    ImposeBranchingDetail::addRangeBranchCutToUnsolved(
                        node, children.back(), brc, pricing_controller.getNumBucketPerVertex(), !IF_SYMMETRY_PROHIBIT);
                    node->setBranchOnM(false);
                    node->setBranchOnN(false);
                }
                if (node->getBranchOnCustomer()){
                    std::cout << "Branching on customer: " << node->getBranchCustomerIdx() << std::endl;
                    ImposeBranchingDetail::addCustomerBranchCutToUnsolved(
                        node, children.back(), brc, pricing_controller.getNumBucketPerVertex(), !IF_SYMMETRY_PROHIBIT);
                    node->setBranchOnCustomer(false);
                }
                
            }
            else
                ImposeBranchingDetail::addBranchCutToUnsolved(
                    node, children.back(), brc, pricing_controller.getNumBucketPerVertex(), !IF_SYMMETRY_PROHIBIT);
        }
        if (children.back() == nullptr) children.pop_back();
        node->clearEdgeMap();
    };



    BbNode::BbNode(BbNode *node,
                   bool if_symmetry,
                   int num_buckets_per_vertex,
                   const Brc &bf) {
        node->if_root_node = false; //start branching
        idx = ++node_idx_counter;
        cols = node->cols;
        // std::cout << "child node cols size = " << cols.size() << std::endl;
        solver.getSolver(&node->solver);
        rccs = node->rccs;
        r1cs = node->r1cs;
        brcs = node->brcs;
        brcs.emplace_back(bf);
        value = node->value;
        num_forward_bucket_arcs = node->num_forward_bucket_arcs;
        num_forward_jump_arcs = node->num_forward_jump_arcs;
        topological_order_forward = node->topological_order_forward;
        if (!if_symmetry) {
            topological_order_backward = node->topological_order_backward;
            num_backward_bucket_arcs = node->num_backward_bucket_arcs;
            num_backward_jump_arcs = node->num_backward_jump_arcs;
        }
        last_gap = node->last_gap;

        all_forward_buckets = new Bucket *[dim];
        if (node->all_forward_buckets == nullptr) std::terminate();
        for (int i = 0; i < dim; ++i) {
            all_forward_buckets[i] = new Bucket[num_buckets_per_vertex];
            for (int j = 0; j < num_buckets_per_vertex; ++j) {
                all_forward_buckets[i][j] = node->all_forward_buckets[i][j];
            }
        }
        if (!if_symmetry) {
            if (node->all_backward_buckets == nullptr) std::terminate();
            all_backward_buckets = new Bucket *[dim];
            for (int i = 0; i < dim; ++i) {
                all_backward_buckets[i] = new Bucket[num_buckets_per_vertex];
                for (int j = 0; j < num_buckets_per_vertex; ++j) {
                    all_backward_buckets[i][j] = node->all_backward_buckets[i][j];
                }
            }
        }
    }


    // BbNode::BbNode(BbNode *node,
    //                bool if_symmetry,
    //                int num_buckets_per_vertex) {
    //     node->if_root_node = false; //start branching
    //     idx = ++node_idx_counter;
    //     cols = node->cols;
    //     std::cout << "child node cols size = " << cols.size() << std::endl;
    //     solver.getSolver(&node->solver);
    //     rccs = node->rccs;
    //     r1cs = node->r1cs;
    //     // brcs = node->brcs;
    //     // brcs.emplace_back(bf);
    //     value = node->value;
    //     num_forward_bucket_arcs = node->num_forward_bucket_arcs;
    //     num_forward_jump_arcs = node->num_forward_jump_arcs;
    //     topological_order_forward = node->topological_order_forward;
    //     if (!if_symmetry) {
    //         topological_order_backward = node->topological_order_backward;
    //         num_backward_bucket_arcs = node->num_backward_bucket_arcs;
    //         num_backward_jump_arcs = node->num_backward_jump_arcs;
    //     }
    //     last_gap = node->last_gap;

    //     all_forward_buckets = new Bucket *[dim];
    //     if (node->all_forward_buckets == nullptr) std::terminate();
    //     for (int i = 0; i < dim; ++i) {
    //         all_forward_buckets[i] = new Bucket[num_buckets_per_vertex];
    //         for (int j = 0; j < num_buckets_per_vertex; ++j) {
    //             all_forward_buckets[i][j] = node->all_forward_buckets[i][j];
    //         }
    //     }
    //     if (!if_symmetry) {
    //         if (node->all_backward_buckets == nullptr) std::terminate();
    //         all_backward_buckets = new Bucket *[dim];
    //         for (int i = 0; i < dim; ++i) {
    //             all_backward_buckets[i] = new Bucket[num_buckets_per_vertex];
    //             for (int j = 0; j < num_buckets_per_vertex; ++j) {
    //                 all_backward_buckets[i][j] = node->all_backward_buckets[i][j];
    //             }
    //         }
    //     }
    // }


    BbNode::BbNode(BbNode *node, const Brc &bf) {
        node->if_root_node = false; //start branching
        if_in_enu_state = true;
        idx = ++node_idx_counter;
        rccs = node->rccs;
        r1cs = node->r1cs;
        brcs = node->brcs;
        last_gap = node->last_gap;
        brcs.emplace_back(bf);

        solver.getSolver(&node->refSolver());
        valid_size = node->valid_size;

        index_columns_in_enumeration_column_pool = node->index_columns_in_enumeration_column_pool;
        cost_for_columns_in_enumeration_column_pool = node->cost_for_columns_in_enumeration_column_pool;
        deleted_columns_in_enumeration_pool.assign(node->deleted_columns_in_enumeration_pool.begin(),
                                                   node->deleted_columns_in_enumeration_pool.begin()
                                                   + node->index_columns_in_enumeration_column_pool.size());
        cols = node->cols;
        value = node->value;
    }
}
