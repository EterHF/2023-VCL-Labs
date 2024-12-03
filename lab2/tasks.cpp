#include <unordered_map>
#include<iostream>
#include <glm/gtc/matrix_inverse.hpp>
#include <spdlog/spdlog.h>

#include "Labs/2-GeometryProcessing/DCEL.hpp"
#include "Labs/2-GeometryProcessing/tasks.h"

namespace VCX::Labs::GeometryProcessing {

#include "Labs/2-GeometryProcessing/marching_cubes_table.h"
    
    glm::vec3 unit(int i){
        if(i==0)return glm::vec3{1.0f,0,0};
        if(i==1)return glm::vec3{0,1.0f,0};
        if(i==2)return glm::vec3{0,0,1.0f};
    }

    glm::vec3 getPoint(glm::vec3 p0,int j,float dx,const std::function<float(const glm::vec3&)>& func){
        glm::vec3 startp=p0+dx*(j&1)*unit(((j>>2)+1)%3)+dx*((j>>1)&1)*unit(((j>>2)+2)%3);
        glm::vec3 endp=startp+unit(j>>2)*dx;
        float v1=func(startp),v2=func(endp);
        return v2/(v2-v1)*startp-v1/(v2-v1)*endp;
        //return 0.5f*(startp+endp);
    }

    /******************* 1. Mesh Subdivision *****************/
    void SubdivisionMesh(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, std::uint32_t numIterations) {
        Engine::SurfaceMesh curr_mesh = input;
        // We do subdivison iteratively.
        for (std::uint32_t it = 0; it < numIterations; ++it) {
            // During each iteration, we first move curr_mesh into prev_mesh.
            Engine::SurfaceMesh prev_mesh;
            prev_mesh.Swap(curr_mesh);
            // Then we create doubly connected edge list.
            DCEL G(prev_mesh);
            if (! G.IsManifold()) {
                spdlog::warn("VCX::Labs::GeometryProcessing::SubdivisionMesh(..): Non-manifold mesh.");
                return;
            }
            // Note that here curr_mesh has already been empty.
            // We reserve memory first for efficiency.
            curr_mesh.Positions.reserve(prev_mesh.Positions.size() * 3 / 2);
            curr_mesh.Indices.reserve(prev_mesh.Indices.size() * 4);
            // Then we iteratively update currently existing vertices.
            for (std::size_t i = 0; i < prev_mesh.Positions.size(); ++i) {
                // Update the currently existing vetex v from prev_mesh.Positions.
                // Then add the updated vertex into curr_mesh.Positions.
                auto v           = G.Vertex(i);
                auto neighbors   = v->Neighbors();
                // your code here:
                int deg=neighbors.size();
                float u=deg==3?3.0f/16.0f:3.0f/8.0f/deg;
                glm::vec3 newp=(1.0f-deg*u)*prev_mesh.Positions[i];
                for(auto each:neighbors){
                    newp+=u*prev_mesh.Positions[each];
                }
                curr_mesh.Positions.push_back(newp);
            }
            // We create an array to store indices of the newly generated vertices.
            // Note: newIndices[i][j] is the index of vertex generated on the "opposite edge" of j-th
            //       vertex in the i-th triangle.
            std::vector<std::array<std::uint32_t, 3U>> newIndices(prev_mesh.Indices.size() / 3, { ~0U, ~0U, ~0U });
            // Iteratively process each halfedge.
            for (auto e : G.Edges()) {
                // newIndices[face index][vertex index] = index of the newly generated vertex
                newIndices[G.IndexOf(e->Face())][e->EdgeLabel()] = curr_mesh.Positions.size();
                auto eTwin                                   = e->TwinEdgeOr(nullptr);
                // eTwin stores the twin halfedge.
                if (! eTwin) {
                    // When there is no twin halfedge (so, e is a boundary edge):
                    // your code here: generate the new vertex and add it into curr_mesh.Positions.
                    glm::vec3 newp={0,0,0};
                    newp+=(prev_mesh.Positions[e->From()]+prev_mesh.Positions[e->To()])*3.5f/8.0f;
                    newp+=prev_mesh.Positions[e->OppositeVertex()]*1.0f/8.0f;
                    curr_mesh.Positions.push_back(newp);
                } else {
                    // When the twin halfedge exists, we should also record:
                    //     newIndices[face index][vertex index] = index of the newly generated vertex
                    // Because G.Edges() will only traverse once for two halfedges,
                    //     we have to record twice.
                    // your code here: generate the new vertex and add it into curr_mesh.Positions.
                    newIndices[G.IndexOf(eTwin->Face())][e->TwinEdge()->EdgeLabel()] = curr_mesh.Positions.size();
                    glm::vec3 newp={0,0,0};
                    newp+=(prev_mesh.Positions[e->From()]+prev_mesh.Positions[e->To()])*3.0f/8.0f;
                    newp+=(prev_mesh.Positions[e->OppositeVertex()]+prev_mesh.Positions[eTwin->OppositeVertex()])*1.0f/8.0f;
                    curr_mesh.Positions.push_back(newp);
                }
            }

            // Here we've already build all the vertices.
            // Next, it's time to reconstruct face indices.
            for (std::size_t i = 0; i < prev_mesh.Indices.size(); i += 3U) {
                // For each face F in prev_mesh, we should create 4 sub-faces.
                // v0,v1,v2 are indices of vertices in F.
                // m0,m1,m2 are generated vertices on the edges of F.
                auto v0           = prev_mesh.Indices[i + 0U];
                auto v1           = prev_mesh.Indices[i + 1U];
                auto v2           = prev_mesh.Indices[i + 2U];
                auto [m0, m1, m2] = newIndices[i / 3U];
                // Note: m0 is on the opposite edge (v1-v2) to v0.
                // Please keep the correct indices order (consistent with order v0-v1-v2)
                //     when inserting new face indices.
                // toInsert[i][j] stores the j-th vertex index of the i-th sub-face.
                std::uint32_t toInsert[4][3] = {
                    // your code here:
                    {v0,m2,m1},
                    {m2,v1,m0},
                    {m1,m2,m0},
                    {m1,m0,v2}
                };
                // Do insertion.
                curr_mesh.Indices.insert(
                    curr_mesh.Indices.end(),
                    reinterpret_cast<std::uint32_t *>(toInsert),
                    reinterpret_cast<std::uint32_t *>(toInsert) + 12U
                );
            }

            if (curr_mesh.Positions.size() == 0) {
                spdlog::warn("VCX::Labs::GeometryProcessing::SubdivisionMesh(..): Empty mesh.");
                output = input;
                return;
            }
        }
        // Update output.
        output.Swap(curr_mesh);
    }

    /******************* 2. Mesh Parameterization *****************/
    void Parameterization(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, const std::uint32_t numIterations) {
        // Copy.
        output = input;
        // Reset output.TexCoords.
        output.TexCoords.resize(input.Positions.size(), glm::vec2 { 0 });

        // Build DCEL.
        DCEL G(input);
        if (! G.IsManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::Parameterization(..): non-manifold mesh.");
            return;
        }
        // Set boundary UVs for boundary vertices.
        // your code here: directly edit output.TexCoords
        int* boundary_idx_seq=new int[output.Positions.size()]{0},cnt=0;
        for (std::size_t i=0; i<output.Positions.size();++i) {
            DCEL::VertexProxy const* v=G.Vertex(i);
            if(v->OnBoundary()){
                boundary_idx_seq[cnt++]=i;
                int idx=0;
                idx=v->BoundaryNeighbors().second;
                while(idx!=i){
                    boundary_idx_seq[cnt++]=idx;
                    v=G.Vertex(idx);
                    idx=v->BoundaryNeighbors().second;
                }
                break;
            }
        }
        int one_forth=cnt/4;
        output.TexCoords[boundary_idx_seq[0]]={0,0};
        output.TexCoords[boundary_idx_seq[one_forth]]={1,0};
        output.TexCoords[boundary_idx_seq[one_forth*2]]={1,1};
        output.TexCoords[boundary_idx_seq[one_forth*3]]={0,1};
        for(int i=1;i<one_forth;++i){
            output.TexCoords[boundary_idx_seq[i]]={i/(double)one_forth,0};
            output.TexCoords[boundary_idx_seq[one_forth+i]]={1,i/(double)one_forth};
            output.TexCoords[boundary_idx_seq[one_forth*2+i]]={1-i/(double)one_forth,1};
        }
        for(int i=1;i<cnt-one_forth*3;++i){
            output.TexCoords[boundary_idx_seq[one_forth*3+i]]={0,1-i/(cnt-(double)one_forth*3)};
        }
        // Solve equation via Gauss-Seidel Iterative Method.
        for (int k = 0; k < numIterations; ++k) {
            // your code here:
            for(int i=0;i<output.Positions.size();++i){
                DCEL::VertexProxy const* v=G.Vertex(i);
                if(!v->OnBoundary()){
                    std::vector<uint32_t> neighbors=v->Neighbors();
                    int num=neighbors.size();
                    double x=0,y=0;
                    for(auto each:neighbors){
                        x+=output.TexCoords[each][0];
                        y+=output.TexCoords[each][1];
                    }
                    output.TexCoords[i]={x/num,y/num};
                }
            }
        }
        delete[] boundary_idx_seq;
    }

    /******************* 3. Mesh Simplification *****************/
    void SimplifyMesh(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, float simplification_ratio) {

        DCEL G(input);
        if (! G.IsManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SimplifyMesh(..): Non-manifold mesh.");
            return;
        }
        // We only allow watertight mesh.
        if (! G.IsWatertight()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SimplifyMesh(..): Non-watertight mesh.");
            return;
        }

        // Copy.
        output = input;

        // Compute Q matrix of the face f.
        auto UpdateQ {
            [&G, &output] (DCEL::Triangle const * f) -> glm::mat4 {
                glm::mat4 Q;
                // your code here:
                if(f==nullptr)return glm::mat4(0.0f);
                glm::vec3 points[3];
                for(int i=0;i<3;++i){
                    auto idx=f->VertexIndex(i);
                    if(G.Vertex(idx)){
                        points[i]=output.Positions[idx];
                    }
                    else return glm::mat4(0.0f);
                }
                glm::vec3 ori_normal=glm::cross(points[1]-points[0],points[2]-points[1]);
                glm::vec3 normal=glm::normalize(ori_normal);
                float d=0.0f-glm::dot(normal,points[0]);
                //printf("%f ",d);
                glm::vec4 p={normal,d};
                for(int i=0;i<4;++i){
                    for(int k=0;k<4;++k){
                        Q[k][i]=p[i]*p[k];
                    }
                }
                return Q;
            }
        };

        // The struct to record constraction info.
        struct ConstractionPair {
            DCEL::HalfEdge const * edge;            // which edge to constract; if $edge == nullptr$, it means this pair is no longer valid
            glm::vec4              targetPosition;  // the targetPosition $v$ for vertex $edge->From()$ to move to
            float                  cost;            // the cost $v.T * Qbar * v$
        };

        // Given an edge (v1->v2), the positions of its two endpoints (p1, p2) and the Q matrix (Q1+Q2),
        //     return the ConstractionPair struct.
        static constexpr auto MakePair {
            [] (DCEL::HalfEdge const * edge,
                glm::vec3 const & p1,
                glm::vec3 const & p2,
                glm::mat4 const & Q
            ) -> ConstractionPair {
                // your code here:
                glm::vec4 tar;
                float cost;
                glm::mat4 q=Q,inv_q;//应该*2.0f吗?
                q[0][3]=0.0f;q[1][3]=0.0f;q[2][3]=0.0f;q[3][3]=1.0f;//注意下标和常识反着来
                if(abs(glm::determinant(q))>=0.001f){//注意加绝对值
                    inv_q=glm::inverse(q);
                    tar=inv_q*glm::vec4{0.0f,0.0f,0.0f,1.0f};
                    glm::vec4 right=Q*tar;
                    cost=glm::dot(tar,right);
                    //printf("%s\n","inv");
                }else{
                    float min_cost=99999.0f,tmp_cost;
                    for(float t=0.0f;t<=1.0f;t+=0.0001f){
                        glm::vec3 tmp=t*p1+(1.0f-t)*p2;
                        glm::vec4 tmp_tar={tmp,1.0f};
                        glm::vec4 right=Q*tmp_tar;
                        tmp_cost=glm::dot(tmp_tar,right);
                        if(tmp_cost<min_cost){
                            tar=tmp_tar;
                            min_cost=tmp_cost;
                        }
                    }
                    cost=min_cost;
                    //printf("%s\n","not_inv");
                }
                return {edge,tar,cost};
            }
        };

        // pair_map: map EdgeIdx to index of $pairs$
        // pairs:    store ConstractionPair
        // Qv:       $Qv[idx]$ is the Q matrix of vertex with index $idx$
        // Qf:       $Qf[idx]$ is the Q matrix of face with index $idx$
        std::unordered_map<DCEL::EdgeIdx, std::size_t> pair_map; 
        std::vector<ConstractionPair>                  pairs; 
        std::vector<glm::mat4>                         Qv(G.NumOfVertices(), glm::mat4(0));
        std::vector<glm::mat4>                         Qf(G.NumOfFaces(),    glm::mat4(0));

        // Initially, we compute Q matrix for each faces and it accumulates at each vertex.
        for (auto f : G.Faces()) {
            auto Q                 = UpdateQ(f);
            Qv[f->VertexIndex(0)] += Q;
            Qv[f->VertexIndex(1)] += Q;
            Qv[f->VertexIndex(2)] += Q;
            Qf[G.IndexOf(f)]       = Q;
        }

        pair_map.reserve(G.NumOfFaces() * 3);
        pairs.reserve(G.NumOfFaces() * 3 / 2);

        // Initially, we make pairs from all the constractable edges.
        for (auto e : G.Edges()) {
            if (! G.IsConstractable(e)) continue;
            auto v1                            = e->From();
            auto v2                            = e->To();
            auto pair                          = MakePair(e, input.Positions[v1], input.Positions[v2], Qv[v1] + Qv[v2]);
            pair_map[G.IndexOf(e)]             = pairs.size();
            pair_map[G.IndexOf(e->TwinEdge())] = pairs.size();
            pairs.emplace_back(pair);
        }

        // Loop until the number of vertices is less than $simplification_ratio * initial_size$.
        while (G.NumOfVertices() > simplification_ratio * Qv.size()) {
            // Find the constractable pair with minimal cost.
            std::size_t min_idx = ~0;
            for (std::size_t i = 1; i < pairs.size(); ++i) {
                if (! pairs[i].edge) continue;
                if (!~min_idx || pairs[i].cost < pairs[min_idx].cost) {
                    if (G.IsConstractable(pairs[i].edge)) min_idx = i;
                    else pairs[i].edge = nullptr;
                }
            }
            if (!~min_idx) break;

            // top:    the constractable pair with minimal cost
            // v1:     the reserved vertex
            // v2:     the removed vertex
            // result: the constract result
            // ring:   the edge ring of vertex v1
            ConstractionPair & top    = pairs[min_idx];
            auto               v1     = top.edge->From();
            auto               v2     = top.edge->To();
            auto               result = G.Constract(top.edge);
            auto               ring   = G.Vertex(v1)->Ring();

            top.edge             = nullptr;            // The constraction has already been done, so the pair is no longer valid. Mark it as invalid.
            output.Positions[v1] = top.targetPosition; // Update the positions.

            // We do something to repair $pair_map$ and $pairs$ because some edges and vertices no longer exist.
            for (int i = 0; i < 2; ++i) {
                DCEL::EdgeIdx removed           = G.IndexOf(result.removed_edges[i].first);
                DCEL::EdgeIdx collapsed         = G.IndexOf(result.collapsed_edges[i].second);
                pairs[pair_map[removed]].edge   = result.collapsed_edges[i].first;
                pairs[pair_map[collapsed]].edge = nullptr;
                pair_map[collapsed]             = pair_map[G.IndexOf(result.collapsed_edges[i].first)];
            }

            //??????WTF??????

            // For the two wing vertices, each of them lose one incident face.
            // So, we update the Q matrix.
            Qv[result.removed_faces[0].first] -= Qf[G.IndexOf(result.removed_faces[0].second)];
            Qv[result.removed_faces[1].first] -= Qf[G.IndexOf(result.removed_faces[1].second)];

            // For the vertex v1, Q matrix should be recomputed.
            // And as the position of v1 changed, all the vertices which are on the ring of v1 should update their Q matrix as well.
            Qv[v1] = glm::mat4(0);
            for (auto e : ring) {
                // your code here:
                //     1. Compute the new Q matrix for $e->Face()$.
                //     2. According to the difference between the old Q (in $Qf$) and the new Q (computed in step 1),
                //        update Q matrix of each vertex on the ring (update $Qv$).
                //     3. Update Q matrix of vertex v1 as well (update $Qv$).
                //     4. Update $Qf$.
                glm::mat4 new_Q=UpdateQ(e->Face());
                glm::mat4 res_Q=new_Q-Qf[G.IndexOf(e->Face())];//bug所在:是e->Face()
                Qv[e->From()]+=res_Q;
                Qv[e->To()]+=res_Q;
                Qv[v1]+=new_Q;
                Qf[G.IndexOf(e->Face())]=new_Q;
            }

            // Finally, as the Q matrix changed, we should update the relative $ConstractionPair$ in $pairs$.
            // Any pair with the Q matrix of its endpoints changed, should be remade by $MakePair$
            // Hint: the official code checks ... rings.
            // your code here:
            for(auto each:ring){
                auto sub_ring=G.Vertex(each->From())->Ring();
                for(auto i:sub_ring){
                    auto idx=pair_map[G.IndexOf(i)];
                    auto p1=i->From(),p2=i->To();
                    if(pairs[idx].edge&&G.Vertex(p1)&&G.Vertex(p2))pairs[idx]=MakePair(i,output.Positions[p1],output.Positions[p2],Qv[p1]+Qv[p2]);
                }
            }
        }

        // In the end, we check if the result mesh is watertight and manifold.
        if (! G.DebugWatertightManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SimplifyMesh(..): Result is not watertight manifold.");
        }

        auto exported = G.ExportMesh();
        output.Indices.swap(exported.Indices);
    }

    /******************* 4. Mesh Smoothing *****************/
    void SmoothMesh(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, std::uint32_t numIterations, float lambda, bool useUniformWeight) {
        // Define function to compute cotangent value of the angle v1-vAngle-v2
        static constexpr auto GetCotangent {
            [] (glm::vec3 vAngle, glm::vec3 v1, glm::vec3 v2) -> float {
                // your code here:
                glm::vec3 e1=v1-vAngle,e2=v2-vAngle;
                float l1=sqrt(glm::dot(e1,e1)),l2=sqrt(glm::dot(e2,e2));
                float product=glm::dot(e1,e2);
                float cos=product/l1/l2;
                float sin=sqrt(1-cos*cos);
                float cot=cos/sin;
                return cot;
            }
        };

        DCEL G(input);
        if (! G.IsManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SmoothMesh(..): Non-manifold mesh.");
            return;
        }
        // We only allow watertight mesh.
        if (! G.IsWatertight()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SmoothMesh(..): Non-watertight mesh.");
            return;
        }

        Engine::SurfaceMesh prev_mesh;
        prev_mesh.Positions = input.Positions;
        for (std::uint32_t iter = 0; iter < numIterations; ++iter) {
            Engine::SurfaceMesh curr_mesh = prev_mesh;
            for (std::size_t i = 0; i < input.Positions.size(); ++i) {
                // your code here: curr_mesh.Positions[i] = ...
                auto neighbors=G.Vertex(i)->Neighbors();
                int n=neighbors.size();
                float sum_weight=0;
                float* weight=new float[n];
                for(int k=0;k<n;++k){
                    if(!useUniformWeight){
                        int idx1=(k+1)%n;
                        int idx2=k-1<0?n-1:k-1;
                        float cot1=GetCotangent(prev_mesh.Positions[neighbors[idx1]],prev_mesh.Positions[i],prev_mesh.Positions[neighbors[k]]);
                        float cot2=GetCotangent(prev_mesh.Positions[neighbors[idx2]],prev_mesh.Positions[i],prev_mesh.Positions[neighbors[k]]);
                        cot1=abs(cot1);
                        cot2=abs(cot2);
                        if(cot1<0.1f)cot1=0;
                        else if(cot1>10.0f)cot1=10.0f;
                        if(cot2<0.1f)cot2=0;
                        else if(cot2>10.0f)cot2=10.0f;
                        // cot1=glm::clamp(cot1,0.1f,5.0f);
                        // cot2=glm::clamp(cot2,0.1f,5.0f);
                        float w=cot1+cot2;
                        weight[k]=w;
                        sum_weight+=w;
                    }else{
                        float w=1.0f;
                        weight[k]=w;
                        sum_weight+=w;
                    }
                }
                glm::vec3 v{0};
                for(int k=0;k<n;++k){
                    v+=weight[k]*prev_mesh.Positions[neighbors[k]];
                }
                v/=sum_weight;
                curr_mesh.Positions[i]=lambda*v+(1.0f-lambda)*prev_mesh.Positions[i];
                delete[] weight;
            }
            // Move curr_mesh to prev_mesh.
            prev_mesh.Swap(curr_mesh);
        }
        // Move prev_mesh to output.
        output.Swap(prev_mesh);
        // Copy indices from input.
        output.Indices = input.Indices;
    }

    /******************* 5. Marching Cubes *****************/
    void MarchingCubes(Engine::SurfaceMesh & output, const std::function<float(const glm::vec3 &)> & sdf, const glm::vec3 & grid_min, const float dx, const int n) {
        // your code here:
        for(int i=0;i<n;++i){
            for(int j=0;j<n;++j){
                for(int k=0;k<n;++k){
                    glm::vec3 pos=grid_min+glm::vec3{i*dx,j*dx,k*dx};//立方体起始点坐标
                    unsigned char idx=0;
                    for(int l=0;l<8;++l){
                        glm::vec3 p=pos+glm::vec3{(l&1)*dx,((l>>1)&1)*dx,(l>>2)*dx};
                        if(sdf(p)>0)idx|=(1<<l);//获取c_EdgeStateTable的下标
                    }
                    for(int l=0;l<16;l+=3){
                        if(c_EdgeOrdsTable[idx][l]!=-1){
                            int v1=c_EdgeOrdsTable[idx][l];
                            int v2=c_EdgeOrdsTable[idx][l+1];
                            int v3=c_EdgeOrdsTable[idx][l+2];
                            glm::vec3 p1=getPoint(pos,v1,dx,sdf);
                            glm::vec3 p2=getPoint(pos,v2,dx,sdf);
                            glm::vec3 p3=getPoint(pos,v3,dx,sdf);
                            uint32_t idx1,idx2,idx3;
                            bool f1=0,f2=0,f3=0;
                            for(size_t m=0;m<output.Positions.size();++m){
                                if(p1==output.Positions[m]){
                                    idx1=m;
                                    f1=1;
                                }
                                if(p2==output.Positions[m]){
                                    idx2=m;
                                    f2=1;
                                }
                                if(p3==output.Positions[m]){
                                    idx3=m;
                                    f3=1;
                                }
                            }
                            if(!f1){
                                output.Positions.push_back(p1);
                                idx1=output.Positions.size()-1;
                            }
                            if(!f2){
                                output.Positions.push_back(p2);
                                idx2=output.Positions.size()-1;
                            }
                            if(!f3){
                                output.Positions.push_back(p3);
                                idx3=output.Positions.size()-1;
                            }
                            output.Indices.push_back(idx1);
                            output.Indices.push_back(idx3);
                            output.Indices.push_back(idx2);
                        }
                        else break;
                    }
                }
            }
        }
    }
} // namespace VCX::Labs::GeometryProcessing
