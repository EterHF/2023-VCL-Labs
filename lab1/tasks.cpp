#include <random>
#include<cmath>
#include <spdlog/spdlog.h>

#include "Labs/1-Drawing2D/tasks.h"

using VCX::Labs::Common::ImageRGB;

template<class T>
void swap(T& a,T& b){
    T tmp=a;
    a=b;
    b=tmp;
}

bool check(int w,int h,int x,int y){
    if(x<0||x>w-1||y<0||y>h-1)return false;
    return true;
}

namespace VCX::Labs::Drawing2D {
    /******************* 1.Image Dithering *****************/
    void DitheringThreshold(
        ImageRGB &       output,
        ImageRGB const & input) {
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                glm::vec3 color = input.At(x, y);
                output.At(x, y) = {
                    color.r > 0.5 ? 1 : 0,
                    color.g > 0.5 ? 1 : 0,
                    color.b > 0.5 ? 1 : 0,
                };
            }
    }

    void DitheringRandomUniform(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        std::mt19937_64 eng{std::random_device{}()};
        std::uniform_real_distribution<double> rd(-0.5,0.5);
        int w=input.GetSizeX(),h=input.GetSizeY();
        //printf("%d %d",output.GetSizeY(),output.GetSizeX());
        for(int i=0;i<w;++i){
            for(int k=0;k<h;++k){
                glm::vec3 tmp=input.At(i,k);
                tmp+=rd(eng);
                output.At(i,k)=glm::vec3{tmp.r>0.5?1:0,tmp.g>0.5?1:0,tmp.b>0.5?1:0,};
            }
        }
    }

    void DitheringRandomBlueNoise(
        ImageRGB &       output,
        ImageRGB const & input,
        ImageRGB const & noise) {
        // your code here:
        int w=input.GetSizeX(),h=input.GetSizeY();
        for(int i=0;i<w;++i){
            for(int k=0;k<h;++k){
                glm::vec3 tmp=input.At(i,k);
                tmp+=noise.At(i,k);
                tmp-=0.5;
                output.At(i,k)=glm::vec3{tmp.r>0.5?1:0,tmp.g>0.5?1:0,tmp.b>0.5?1:0,};
            }
        }
    }

    void DitheringOrdered(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        int order[3][3]={{6,8,4},{1,0,3},{5,2,7}};
        int w=input.GetSizeX(),h=input.GetSizeY();
        for(int i=0;i<w;++i){
            for(int k=0;k<h;++k){
                int x0=3*i,y0=3*k;
                double color=input.At(i,k)[0];
                for(int x=0;x<3;++x){
                    for(int y=0;y<3;++y){
                        int outcome=color>=(order[x][y]/9.0)?1:0;
                        output.At(x0+x,y0+y)=glm::vec3(outcome);
                    }
                }
            }
        }
    }

    void DitheringErrorDiffuse(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        int w=input.GetSizeX(),h=input.GetSizeY();
        output=input;
        for(int k=0;k<h;++k){
            for(int i=0;i<w;++i){
                glm::vec3 color=output.At(i,k);
                double thres=color[0]>0.5?1:0;
                output.At(i,k)=glm::vec3(thres);
                double error=color[0]-thres;
                double e1=error*7/16,e2=error*3/16,e3=error*5/16,e4=error/16;
                if(check(w,h,i-1,k+1))output.At(i-1,k+1)=(glm::vec3)output.At(i-1,k+1)+glm::vec3(e2);
                if(check(w,h,i,k+1))output.At(i,k+1)=(glm::vec3)output.At(i,k+1)+glm::vec3(e3);
                if(check(w,h,i+1,k+1))output.At(i+1,k+1)=(glm::vec3)output.At(i+1,k+1)+glm::vec3(e4);
                if(check(w,h,i+1,k))output.At(i+1,k)=(glm::vec3)output.At(i+1,k)+glm::vec3(e1);
            }
        }
    }

    /******************* 2.Image Filtering *****************/
    void Blur(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        int w=input.GetSizeX(),h=input.GetSizeY();
        double filter[3][3]={0};
        for(int i=0;i<3;++i)
            for(int k=0;k<3;++k)
                filter[i][k]=1.0/9;
        for(int i=0;i<w;++i){
            for(int k=0;k<h;++k){
                glm::vec3 sum=glm::vec3(0);
                int cnt=0;
                for(int x=-1;x<2;++x){
                    for(int y=-1;y<2;++y){
                        if(check(h,w,i+x,k+y)){
                            sum+=(glm::vec3)input.At(i+x,k+y);
                            cnt++;
                        }
                    }
                }
                output.At(i,k)=sum/glm::vec3(cnt);
            }
        }
    }

    void Edge(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        int w=input.GetSizeX(),h=input.GetSizeY();
        double filter1[3][3]={{-1,0,1},{-2,0,2},{-1,0,1}},filter2[3][3]={{-1,-2,-1},{0,0,0},{1,2,1}};
        for(int i=0;i<w;++i){
            for(int k=0;k<h;++k){
                double* sum1=new double[3]{0},*sum2=new double[3]{0},*sum=new double[3]{0};
                for(int x=-1;x<2;++x){
                    for(int y=-1;y<2;++y){
                        glm::vec3 color=check(h,w,i+x,k+y)?input.At(i+x,k+y):glm::vec3(0);
                        for(int cnt=0;cnt<3;++cnt){
                            sum1[cnt]+=color[cnt]*filter1[x+1][y+1];
                            sum2[cnt]+=color[cnt]*filter2[x+1][y+1];
                        }
                        
                    }
                }
                for(int cnt=0;cnt<3;++cnt){
                    sum[cnt]+=sqrt(sum1[cnt]*sum1[cnt]+sum2[cnt]*sum2[cnt]);
                }
                output.At(i,k)=glm::vec3{sum[0],sum[1],sum[2]};
                delete[] sum1;
                delete[] sum2;
                delete[] sum;
            }
        }
    }

    /******************* 3. Image Inpainting *****************/
    void Inpainting(
        ImageRGB &         output,
        ImageRGB const &   inputBack,
        ImageRGB const &   inputFront,
        const glm::ivec2 & offset) {
        output             = inputBack;
        std::size_t width  = inputFront.GetSizeX();
        std::size_t height = inputFront.GetSizeY();
        glm::vec3 * g      = new glm::vec3[width * height];
        memset(g, 0, sizeof(glm::vec3) * width * height);
        // set boundary condition
        for (std::size_t y = 0; y < height; ++y) {
            // set boundary for (0, y), your code: g[y * width] = ?
            g[y*width]=(glm::vec3)output.At(offset.x,y+offset.y)-(glm::vec3)inputFront.At(0,y);
            // set boundary for (width - 1, y), your code: g[y * width + width - 1] = ?
            g[y*width+width-1]=(glm::vec3)output.At(offset.x+width-1,y+offset.y)-(glm::vec3)inputFront.At(width-1,y);
        }
        for (std::size_t x = 0; x < width; ++x) {
            // set boundary for (x, 0), your code: g[x] = ?
            g[x]=(glm::vec3)output.At(x+offset.x,offset.y)-(glm::vec3)inputFront.At(x,0);
            // set boundary for (x, height - 1), your code: g[(height - 1) * width + x] = ?
            g[(height-1)*width+x]=(glm::vec3)output.At(x+offset.x,offset.y+height-1)-(glm::vec3)inputFront.At(x,height-1);
        }
        //利用拉普拉斯算子作用在(f-g)上应该等于0，然后运用Gauss-Seidel迭代法求解方程组(确保边界完美融合并保持不变)
        // Jacobi iteration, solve Ag = b
        for (int iter = 0; iter < 8000; ++iter) {
            for (std::size_t y = 1; y < height - 1; ++y)
                for (std::size_t x = 1; x < width - 1; ++x) {
                    g[y * width + x] = (g[(y - 1) * width + x] + g[(y + 1) * width + x] + g[y * width + x - 1] + g[y * width + x + 1]);
                    g[y * width + x] = g[y * width + x] * glm::vec3(0.25);
                }
        }

        for (std::size_t y = 0; y < inputFront.GetSizeY(); ++y)
            for (std::size_t x = 0; x < inputFront.GetSizeX(); ++x) {
                glm::vec3 color = g[y * width + x] + inputFront.At(x, y);
                output.At(x + offset.x, y + offset.y) = color;
            }
        delete[] g;
    }

    /******************* 4. Line Drawing *****************/
    void DrawLine(
        ImageRGB &       canvas,
        glm::vec3 const  color,
        glm::ivec2 const p0,
        glm::ivec2 const p1) {
        // your code here:
        int x0=p0.x,y0=p0.y,x1=p1.x,y1=p1.y,y_dir=1;//y_dir:坐标增长方向
        bool reverse=false;//是否翻转x、y坐标
        if(abs(y1-y0)>abs(x1-x0)){
            reverse=true;
            swap(x0,y0);
            swap(x1,y1);
        }
        if(x0>x1){//x从小到大遍历
            swap(x0,x1);
            swap(y0,y1);
        }
        if(y0>y1){
            y_dir=-1;y0*=-1;y1*=-1;//y坐标取反
        }
        int dx=2*(x1-x0);
        int dy=2*(y1-y0);
        int y=y0,delta=dy-dx,error=dy-dx/2;//初始化error
        for(int i=x0;i<=x1;++i){
            if(reverse)canvas.At(y*y_dir,i)=color;
            else canvas.At(i,y*y_dir)=color; 
            if(error<0)error+=dy;
            else{
                y++;;
                error+=delta;
            }
        }
    }

    /******************* 5. Triangle Drawing *****************/
    void DrawTriangleFilled(
        ImageRGB &       canvas,
        glm::vec3 const  color,
        glm::ivec2 const p0,
        glm::ivec2 const p1,
        glm::ivec2 const p2) {
        // your code here:
        double x[3]={p0.x,p1.x,p2.x},y[3]={p0.y,p1.y,p2.y};
        for(int i=0;i<2;++i){
            for(int k=i+1;k<3;++k){
                if(y[i]>y[k]){
                    swap(y[i],y[k]);
                    swap(x[i],x[k]);
                }
            }
        }//按照y坐标从小到大排序
        if((y[0]-y[1])*(x[1]-x[2])==(y[1]-y[2])*(x[0]-x[1])){
            DrawLine(canvas,color,glm::ivec2((int)x[0],(int)y[0]),glm::ivec2((int)x[2],(int)y[2]));
            return;
        }//三点共线
        bool up_flat=(y[0]==y[1]);//三角形顶部是否水平
        if(!up_flat){
            double dx1=(x[1]-x[0])/(y[1]-y[0]),dx2=(x[2]-x[0])/(y[2]-y[0]);
            double x1=x[0],x2=x[0];
            for(int i=(int)y[0];i<(int)y[1];++i){
                int x_L=std::min(x1,x2),x_R=std::max(x1,x2);
                for(int k=x_L;k<=x_R;++k)canvas.At(k,i)=color;
                x1+=dx1;
                x2+=dx2;
            }
            if(y[1]!=y[2])dx1=(x[2]-x[1])/(y[2]-y[1]);
            for(int i=(int)y[1];i<=(int)y[2];++i){
                int x_L=std::min(x1,x2),x_R=std::max(x1,x2);
                for(int k=x_L;k<=x_R;++k)canvas.At(k,i)=color;
                x1+=dx1;
                x2+=dx2;
            }
        }
        else{
            if(x[0]>x[1]){swap(x[0],x[1]);swap(y[0],y[1]);}
            double dx1=(x[2]-x[0])/(y[2]-y[0]),dx2=(x[2]-x[1])/(y[2]-y[1]);
            double x1=x[0],x2=x[1];
            for(int i=(int)y[0];i<=(int)y[2];++i){
                int x_L=std::min(x1,x2),x_R=std::max(x1,x2);
                for(int k=x_L;k<=x_R;++k)canvas.At(k,i)=color;
                x1+=dx1;
                x2+=dx2;
            }
        }
    }

    /******************* 6. Image Supersampling *****************/
    void Supersample(
        ImageRGB &       output,
        ImageRGB const & input,
        int              rate) {
        // your code here:
        //printf("%d %d",output.GetSizeX(),output.GetSizeY());
        //printf("%d %d",input.GetSizeX(),input.GetSizeY());
        int inputX=input.GetSizeX(),inputY=input.GetSizeY();
        int outputX=output.GetSizeX(),outputY=output.GetSizeY();
        double k1=inputX/outputX,k2=inputY/outputY;
        for(int x=0;x<outputX;++x){
            for(int y=0;y<outputY;++y){
                int x1=x*k1,y1=y*k2;
                double stride1=k1/rate,stride2=k2/rate;
                double cnt=0;
                double r=0,g=0,b=0;
                for(int i=0;i<rate;++i){
                    for(int k=0;k<rate;++k){
                        //double ee=(i-rate/2.0)*(i-rate/2.0)+(k-rate/2.0)*(k-rate/2.0);
                        //double sigma=2;
                        //double weight=exp(-ee/sigma)/3.14/sigma;//Gaussian
                        glm::vec3 rgb=input.At((int)(x1+(i+0.5)*stride1),(int)(y1+(k+0.5)*stride2));
                        r+=rgb[0];g+=rgb[1];b+=rgb[2];
                        cnt++;
                    }
                }
                output.At(x,y)=glm::vec3{r/cnt,g/cnt,b/cnt};;
            }
        }
    }

    /******************* 7. Bezier Curve *****************/
    // Note: Please finish the function [DrawLine] before trying this part.
    glm::vec2 CalculateBezierPoint(
        std::span<glm::vec2> points,
        float const          t) {
        // your code here:
        int len=points.size();
        double m=1-t;
        std::vector<double> x,y;
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
        return glm::vec2((int)x[0],y[0]);
    }
} // namespace VCX::Labs::Drawing2D