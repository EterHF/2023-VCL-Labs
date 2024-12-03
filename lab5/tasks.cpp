#include "Labs/5-Visualization/tasks.h"

#include <numbers>
#include<cstring>

using VCX::Labs::Common::ImageRGB;
namespace VCX::Labs::Visualization {
    std::string names[7]={"cylinders","displacement","weight","horsepower","acceleration","mileage","year"};
    std::string mins[7]={"2","29","1260","27","6","5","68"};
    std::string maxs[7]={"9","494","5493","249","27","51","84"};
    float minf[7]={2,29,1260,27,6,5,68};
    float maxf[7]={9,494,5493,249,27,51,84};
    float range[7]={7,465,4233,222,21,46,16};
    float pos[7]{0};

    struct CoordinateStates {
        // your code here
        std::vector<Car> data;
        CoordinateStates(std::vector<Car> const & i){data=i;}
        bool Update(InteractProxy const & proxy){
            return false;
        }
        void Paint(Common::ImageRGB & input){
            int cnt=0;
            SetBackGround(input,glm::vec4(0.96));
            for(auto each:data){
                pos[0]=(each.cylinders-minf[0])/range[0];
                pos[1]=(each.displacement-minf[1])/range[1];
                pos[2]=(each.weight-minf[2])/range[2];
                pos[3]=(each.horsepower-minf[3])/range[3];
                pos[4]=(each.acceleration-minf[4])/range[4];
                pos[5]=(each.mileage-minf[5])/range[5];
                pos[6]=(each.year-minf[6])/range[6];
                for(int i=0;i<6;++i){
                    DrawLine(input,glm::vec4(1.0-std::min(1.0,0.002*cnt),0.3f,std::min(1.0,0.002*cnt),0.8),
                            glm::vec2{0.05+i*0.15,0.97-pos[i]*0.92},
                            glm::vec2{0.05+(i+1)*0.15,0.97-pos[i+1]*0.92},1);
                }
                cnt++;
            }
            for(int i=0;i<7;++i){
                DrawFilledRect(input,glm::vec4{0.001f,0.001f,0.001f,0.1f},glm::vec2{0.0425+i*0.15,0.05},glm::vec2{0.015,0.92});
                DrawRect(input,glm::vec4{1.f,1.f,1.,0.6f},glm::vec2{0.0425+i*0.15,0.05},glm::vec2{0.015,0.92},1);
                DrawLine(input,glm::vec4(0.4f,0.4f,0.4f,0.5f),glm::vec2{0.05+i*0.15,0.05},glm::vec2{0.05+i*0.15,0.97},2);
                PrintText(input,glm::vec4(0,0,0,1),glm::vec2{0.05+i*0.15,0.01},0.02,names[i]);
                PrintText(input,glm::vec4(0,0,0,1),glm::vec2{0.05+i*0.15,0.035},0.02,maxs[i]);
                PrintText(input,glm::vec4(0,0,0,1),glm::vec2{0.05+i*0.15,0.98},0.02,mins[i]);
            }
        }
    };

    bool PaintParallelCoordinates(Common::ImageRGB & input, InteractProxy const & proxy, std::vector<Car> const & data, bool force) {
        // your code here
        // for example: 
        //   static CoordinateStates states(data);
        //   SetBackGround(input, glm::vec4(1));
        //   ...
        static CoordinateStates states(data); // initialize
        bool change = states.Update(proxy); // update according to user input
        if (! force && ! change) return false; // determine to skip repainting
        states.Paint(input); // visualize
        return true;
    }

    float d(float x,float y,float dx,float dy){
        float dt_x=0,dt_y=0;
        if(dy>0)dt_y=((glm::floor(y)+1)-y)/dy;
        else if(dy<0)dt_y=(y-(glm::ceil(y)-1))/-dy;
        if(dx>0)dt_x=((glm::floor(x)+1)-x)/dx;
        else if(dx<0)dt_x=(x-(glm::ceil(x)-1))/-dx;
        if(dx==0&&dy==0)return 0;
        return std::min(dt_x,dt_y);
    }

    void LIC(ImageRGB & output, Common::ImageRGB const & noise, VectorField2D const & field, int const & step) {
        // your code here
        int h=field.size.first;
        int w=field.size.second;
        output=Common::CreatePureImageRGB(h,w,glm::vec3(0));
        float* weight=new float[2*step+2];
        float sum_weight=0;
        for(int i=1;i<=2*step+1;++i){
            weight[i]=glm::exp(-(i-step-1)*(i-step-1)/step/step/8.f)/2.507f/2.f;
            //weight[i]=glm::pow(glm::cos(0.46f*(i-step-1)),2);
            sum_weight+=weight[i];
        }
        for(int i=0;i<h;++i){
            for(int k=0;k<w;++k){
                float y=i;//row
                float x=k;//col
                glm::vec3 sum=glm::vec3(0);
                for(int j=0;j<step;++j){
                    glm::vec2 v=field.At(int(y),int(x));
                    float dt=d(x,y,v.x,v.y);
                    y+=dt*v.y;
                    x+=dt*v.x;
                    y=glm::clamp(y,0.f,h-1.f);
                    x=glm::clamp(x,0.f,w-1.f);
                    sum+=noise.At(int(y),int(x))*weight[step-j];
                }
                y=i;
                x=k;
                for(int j=0;j<step;++j){
                    glm::vec2 v=field.At(int(y),int(x));
                    float dt=d(x,y,-v.x,-v.y);
                    y+=dt*-v.y;
                    x+=dt*-v.x;
                    y=glm::clamp(y,0.f,h-1.f);
                    x=glm::clamp(x,0.f,w-1.f);
                    sum+=noise.At(int(y),int(x))*weight[step+2+j];
                }
                output.At(i,k)=(noise.At(i,k)*weight[step+1]+sum)/sum_weight;
            }
        }
        delete[]weight;
    }
}; // namespace VCX::Labs::Visualization