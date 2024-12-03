#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <spdlog/spdlog.h>
#include <iostream>
#include "Labs/4-Animation/tasks.h"
#include "IKSystem.h"
#include "CustomFunc.inl"


namespace VCX::Labs::Animation {
    void ForwardKinematics(IKSystem & ik, int StartIndex) {
        if (StartIndex == 0) {
            ik.JointGlobalRotation[0] = ik.JointLocalRotation[0];
            ik.JointGlobalPosition[0] = ik.JointLocalOffset[0];
            StartIndex                = 1;
        }
        
        for (int i = StartIndex; i < ik.JointLocalOffset.size(); i++) {
            // your code here: forward kinematics, update JointGlobalPosition and JointGlobalRotation
            // ik.JointLocalOffset[i]=ik.JointGlobalRotation[i-1]*ik.JointLocalOffset[i];//不更新Offset,累乘旋转,从原始Offset变换
            ik.JointGlobalRotation[i]=ik.JointLocalRotation[i]*ik.JointGlobalRotation[i-1];//Rotation表示后继骨骼旋转
            ik.JointGlobalPosition[i]=ik.JointGlobalPosition[i-1]+ik.JointGlobalRotation[i-1]*ik.JointLocalOffset[i];//Offset表示相对父关节偏移量
        }
    }

    void InverseKinematicsCCD(IKSystem & ik, const glm::vec3 & EndPosition, int maxCCDIKIteration, float eps) {
        ForwardKinematics(ik, 0);
        // These functions will be useful: glm::normalize, glm::rotation, glm::quat * glm::quat
        for (int CCDIKIteration = 0; CCDIKIteration < maxCCDIKIteration && glm::l2Norm(ik.EndEffectorPosition() - EndPosition) > eps; CCDIKIteration++) {
            // your code here: ccd ik
            int index=ik.NumJoints()-2-CCDIKIteration%(ik.NumJoints()-1);
            glm::vec3 endline=EndPosition-ik.JointGlobalPosition[index];
            glm::vec3 startline=ik.EndEffectorPosition()-ik.JointGlobalPosition[index];
            glm::quat r=glm::rotation(glm::normalize(startline),glm::normalize(endline));
            ik.JointLocalRotation[index]=r*ik.JointLocalRotation[index];//ForwardKinematics中不更新Offset,则需要累乘旋转
            ForwardKinematics(ik,index);
            // if(index==0){
            //     for(int i=0;i<ik.NumJoints();++i){
            //         ik.JointGlobalRotation[i]=glm::quat(1,0,0,0);
            //         ik.JointLocalRotation[i]=glm::quat(1,0,0,0);
            //     }
            // }//若ForwardKinematics中更新Offset，则此处需要重置
        }
    }

    void InverseKinematicsFABR(IKSystem & ik, const glm::vec3 & EndPosition, int maxFABRIKIteration, float eps) {
        ForwardKinematics(ik, 0);
        int nJoints = ik.NumJoints();
        std::vector<glm::vec3> backward_positions(nJoints, glm::vec3(0, 0, 0)), forward_positions(nJoints, glm::vec3(0, 0, 0));
        for (int IKIteration = 0; IKIteration < maxFABRIKIteration && glm::l2Norm(ik.EndEffectorPosition() - EndPosition) > eps; IKIteration++) {
            // task: fabr ik
            // backward update
            glm::vec3 next_position         = EndPosition;
            backward_positions[nJoints - 1] = EndPosition;

            for (int i = nJoints - 2; i >= 0; i--) {
                // your code here
                backward_positions[i]=backward_positions[i+1]-ik.JointOffsetLength[i+1]*glm::normalize(backward_positions[i+1]-ik.JointGlobalPosition[i]);
            }

            // forward update
            glm::vec3 now_position = ik.JointGlobalPosition[0];
            forward_positions[0] = ik.JointGlobalPosition[0];
            for (int i = 0; i < nJoints - 1; i++) {
                // your code here
                forward_positions[i+1]=forward_positions[i]+ik.JointOffsetLength[i+1]*glm::normalize(backward_positions[i+1]-forward_positions[i]);
                ik.JointLocalOffset[i+1]=forward_positions[i+1]-forward_positions[i];//Offset也要做出对应更新！！！
            }
            ik.JointGlobalPosition = forward_positions; // copy forward positions to joint_positions
        }

        // Compute joint rotation by position here.
        for (int i = 0; i < nJoints - 1; i++) {
            ik.JointGlobalRotation[i] = glm::rotation(glm::normalize(ik.JointLocalOffset[i + 1]), glm::normalize(ik.JointGlobalPosition[i + 1] - ik.JointGlobalPosition[i]));
        }
        ik.JointLocalRotation[0] = ik.JointGlobalRotation[0];
        for (int i = 1; i < nJoints - 1; i++) {
            ik.JointLocalRotation[i] = glm::inverse(ik.JointGlobalRotation[i - 1]) * ik.JointGlobalRotation[i];
        }
        ForwardKinematics(ik, 0);
    }

    glm::vec2 CalculateBezierPoint(
        std::vector<glm::vec2> points,
        float const          t) {
        // your code here:
        int len=points.size();
        float m=1-t;
        std::vector<float> x,y;
        for(int i=len-1;i>0;--i){
            for(int k=0;k<i;++k){
                if(i==len-1){
                    x.push_back(m*points[k].x+(1-m)*points[k+1].x);
                    y.push_back(m*points[k].y+(1-m)*points[k+1].y);
                    continue;
                }
                x[k]=m*x[k]+(1-m)*x[k+1];
                y[k]=m*y[k]+(1-m)*y[k+1];
            }
        }
        return glm::vec2(x[0],y[0]);
    }

    // IKSystem::Vec3ArrPtr IKSystem::BuildCustomTargetPosition() {
    //     // get function from https://www.wolframalpha.com/input/?i=Albert+Einstein+curve
    //     int nums = 5000;
    //     using Vec3Arr = std::vector<glm::vec3>;
    //     std::shared_ptr<Vec3Arr> custom(new Vec3Arr(nums));
    //     int index = 0;
    //     for (int i = 0; i < nums; i++) {
    //         float x_val = 1.5e-3f * custom_x(92 * glm::pi<float>() * i / nums);
    //         float y_val = 1.5e-3f * custom_y(92 * glm::pi<float>() * i / nums);
    //         if (std::abs(x_val) < 1e-3 || std::abs(y_val) < 1e-3) continue;
    //         (*custom)[index++] = glm::vec3(1.6f - x_val, 0.0f, y_val - 0.2f);
    //     }
    //     custom->resize(index);
    //     return custom;
    // }

    IKSystem::Vec3ArrPtr IKSystem::BuildCustomTargetPosition() {
        // get function from https://www.wolframalpha.com/input/?i=Albert+Einstein+curve
        int nums = 15;
        using Vec3Arr = std::vector<glm::vec3>;
        std::shared_ptr<Vec3Arr> custom(new Vec3Arr(4*nums+4));
        int index = 0;
        std::vector<glm::vec2>  v1{{0.5f,0.8f},{0.6f,0.3f},{0.7f,0.8f}},
                                v2{{0.6f,0.5f},{0.6f,0.4f},{0.6f,0.3f}},
                                v3{{0.9f,0.3f},{1.0f,1.0f},{1.1f,0.3f}},
                                v4{{1.1f,0.3f},{1.2f,1.0f},{1.3f,0.3f}};
        std::vector<std::vector<glm::vec2> > v;
        v.push_back(v1);v.push_back(v2);v.push_back(v3);v.push_back(v4);
        for(int k=0;k<4;++k){
            for (int i = 0; i <= nums; i++) {
                // float x_val = 1.5e-3f * custom_x(92 * glm::pi<float>() * i / nums);
                // float y_val = 1.5e-3f * custom_y(92 * glm::pi<float>() * i / nums);
                //std::cout<<x_val<<std::endl<<y_val<<std::endl;
                glm::vec2 xy=CalculateBezierPoint(v[k],(float)i/nums);
                float x_val=xy[0],y_val=xy[1];
                if (std::abs(x_val) < 1e-3 || std::abs(y_val) < 1e-3) continue;
                (*custom)[index++] = glm::vec3(1.6f - x_val, 0.0f, y_val - 0.2f);
            }
        }
        custom->resize(index);
        return custom;
    }

    static Eigen::VectorXf glm2eigen(std::vector<glm::vec3> const & glm_v) {
        Eigen::VectorXf v = Eigen::Map<Eigen::VectorXf const, Eigen::Aligned>(reinterpret_cast<float const *>(glm_v.data()), static_cast<int>(glm_v.size() * 3));
        return v;
    }

    static std::vector<glm::vec3> eigen2glm(Eigen::VectorXf const & eigen_v) {
        return std::vector<glm::vec3>(
            reinterpret_cast<glm::vec3 const *>(eigen_v.data()),
            reinterpret_cast<glm::vec3 const *>(eigen_v.data() + eigen_v.size())
        );
    }

    static Eigen::SparseMatrix<float> CreateEigenSparseMatrix(std::size_t n, std::vector<Eigen::Triplet<float>> const & triplets) {
        Eigen::SparseMatrix<float> matLinearized(n, n);
        matLinearized.setFromTriplets(triplets.begin(), triplets.end());
        return matLinearized;
    }

    // solve Ax = b and return x
    static Eigen::VectorXf ComputeSimplicialLLT(
        Eigen::SparseMatrix<float> const & A,
        Eigen::VectorXf const & b) {
        auto solver = Eigen::SimplicialLLT<Eigen::SparseMatrix<float>>(A);
        return solver.solve(b);
    }

    void AdvanceMassSpringSystem(MassSpringSystem & system, float const dt) {
        // your code here: rewrite following code
        // int const steps = 1000;
        // float const ddt = dt / steps; 
        // for (std::size_t s = 0; s < steps; s++) {

            // std::vector<glm::vec3> forces(system.Positions.size(), glm::vec3(0));
            // for (auto const spring : system.Springs) {
            //     auto const p0 = spring.AdjIdx.first;
            //     auto const p1 = spring.AdjIdx.second;
            //     glm::vec3 const x01 = system.Positions[p1] - system.Positions[p0];
            //     glm::vec3 const v01 = system.Velocities[p1] - system.Velocities[p0];
            //     glm::vec3 const e01 = glm::normalize(x01);
            //     glm::vec3 f = (system.Stiffness * (glm::length(x01) - spring.RestLength) + system.Damping * glm::dot(v01, e01)) * e01;
            //     forces[p0] += f;
            //     forces[p1] -= f;
            // }
            // for (std::size_t i = 0; i < system.Positions.size(); i++) {
            //     if (system.Fixed[i]) continue;
            //     system.Velocities[i] += (glm::vec3(0, -system.Gravity, 0) + forces[i] / system.Mass) * ddt;
            //     system.Positions[i] += system.Velocities[i] * ddt;
            // }


        float const h=dt;
        std::size_t n=system.Positions.size();
        
        Eigen::VectorXf xk=glm2eigen(system.Positions);
        Eigen::VectorXf vk=glm2eigen(system.Velocities);
        std::vector<glm::vec3> fext(n,glm::vec3(0,-system.Gravity,0));
        Eigen::VectorXf deltak=-h*vk-h*h*glm2eigen(fext);
        
        std::vector<glm::vec3> grad_E(n,glm::vec3(0));
        Eigen::SparseMatrix<float> Hgxk(3*n,3*n);
        std::vector<Eigen::Triplet<float>> triplets2;
        for(auto const spring:system.Springs){
            auto const xi=spring.AdjIdx.first;
            auto const xj=spring.AdjIdx.second;
            glm::vec3 const eij=system.Positions[xj]-system.Positions[xi];
            glm::vec3 const nij=glm::normalize(eij);
            float len=glm::length(eij);
            float len2=glm::length2(eij);
            
            glm::vec3 const grad=system.Stiffness*(len-spring.RestLength)*nij;
            grad_E[xi]+=(-grad);
            grad_E[xj]+=grad;

            glm::mat3 xij;
            for(int i=0;i<3;++i){//row
                for(int k=0;k<3;++k){//col
                    xij[k][i]=nij[k]*nij[i];
                }
            }
            glm::mat3 He=system.Stiffness*xij+system.Stiffness*(1-spring.RestLength/len)*(glm::mat3(1.0f)-xij);
            std::vector<Eigen::Triplet<float>> triplets1;
            for(int i=0;i<3;++i){//row
                for(int k=0;k<3;++k){//col
                    triplets1.emplace_back(xi*3+i,xi*3+k,He[k][i]);
                    triplets1.emplace_back(xj*3+i,xj*3+k,He[k][i]);
                    triplets1.emplace_back(xi*3+i,xj*3+k,-He[k][i]);
                    triplets1.emplace_back(xj*3+i,xi*3+k,-He[k][i]);
                }
            }
            Hgxk+=CreateEigenSparseMatrix(3*n, triplets1);
        }
        for(int i=0;i<3*n;++i){
            triplets2.emplace_back(i,i,system.Mass);
        }
        auto M=CreateEigenSparseMatrix(3*n,triplets2);
        Hgxk+=M/h/h;
        Eigen::VectorXf Exk=glm2eigen(grad_E);
        Eigen::VectorXf gxk=system.Mass*deltak/h/h+Exk;
        Eigen::VectorXf res_x=ComputeSimplicialLLT(Hgxk,-gxk);
        std::vector<glm::vec3> res=eigen2glm(res_x);
        for (std::size_t i=0;i<n;++i) {
            if(system.Fixed[i])continue;
            system.Positions[i]+=res[i];
            system.Velocities[i]=res[i]/h;
        }
        // std::vector<glm::vec3> forces(n,glm::vec3(0));
        // for (auto const spring:system.Springs) {
        //     auto const xi=spring.AdjIdx.first;
        //     auto const xj=spring.AdjIdx.second;
        //     glm::vec3 const eij=system.Positions[xj]-system.Positions[xi];
        //     glm::vec3 f=glm::vec3(system.Stiffness*(glm::length(eij)-spring.RestLength));
        //     forces[xi]+=f;
        //     forces[xj]-=f;
        // }
        // }
    }
}
