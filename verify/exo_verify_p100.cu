// exo_verify_p100.cu — 1+2+3 助力算法验证 · P100 批量版 (10^6 次/热图)
// 设计: 一个线程跑一整条试验(3000 步 × 1ms)。热图 = 参数网格 × 每格重复。
//   mode 0: G × k_fric            → 失败率
//   mode 1: 延迟 × ESO带宽        → 失败率 (故意不施加带宽约束, 看发散区)
//   mode 2: θ0误差 × mgl误差      → 重力保持失败率
#include <cstdio>
#include <cstdlib>
#include <cuda_runtime.h>

static const double DT = 1e-3;
static const int NSTEP = 3000;

struct Cp { double mgl,theta0,fc,bv,k_fric,G,w0,tlimit,ff_limit,total_limit,deadzone,flpf,alpf,hp,J_hat; };
struct Cs { double z1,z2,z3,fric,assist,hp_s,tau_prev; };

__device__ __forceinline__ double clampd(double v,double lo,double hi){return v<lo?lo:(v>hi?hi:v);}

__device__ double ctrl(const Cp*p, Cs*s, double theta, double omega){
    double w=omega, sgn=(w>1e-4)?1.0:(w<-1e-4?-1.0:0.0);
    double fr=p->k_fric*(p->fc*sgn + p->bv*w);
    if (fabs(w)<p->deadzone) fr=0.0;
    s->fric += p->flpf*(fr - s->fric);
    double tg = p->mgl*cos(theta - p->theta0);
    double b0=1.0/p->J_hat;
    double e=s->z1-theta;
    double b1=3*p->w0,b2=3*p->w0*p->w0,b3=p->w0*p->w0*p->w0;
    s->z1+=DT*(s->z2-b1*e);
    s->z2+=DT*(s->z3+b0*s->tau_prev-b2*e);
    s->z3+=DT*(-b3*e);
    if (!isfinite(s->z1)||!isfinite(s->z2)||!isfinite(s->z3)||fabs(s->z3)>1e4||fabs(s->z2)>1e4)
        { s->z1=theta; s->z2=omega; s->z3=0.0; }
    double te=p->J_hat*s->z3;
    s->hp_s += p->hp*(te - s->hp_s);
    double thd = te - s->hp_s;
    s->assist += p->alpf*(p->G*thd - s->assist);
    double ff = clampd(tg + s->fric, -p->ff_limit, p->ff_limit);
    double ta = clampd(s->assist, -p->tlimit, p->tlimit);
    double out = clampd(ff+ta, -p->total_limit, p->total_limit);
    s->tau_prev=out;
    return out;
}

/* 计数器哈希 RNG */
__device__ __forceinline__ unsigned long long xs(unsigned long long*s){
    unsigned long long x=*s; x^=x<<13; x^=x>>7; x^=x<<17; *s=x; return x;
}
__device__ __forceinline__ double urand(unsigned long long*s){ return (xs(s)>>11)*(1.0/9007199254740992.0); }
__device__ __forceinline__ double nrand(unsigned long long*s){   // Box-Muller
    double u1=urand(s)+1e-12, u2=urand(s);
    return sqrt(-2.0*log(u1))*cos(6.283185307179586*u2);
}

/* 返回: bit0=NaN, bit1=失控, bit2=自激, bit3=重力保持失效, bit4=超限幅 */
__device__ int run_trial(Cp cp, double G, double k_fric, double delay_ms, double w0,
                         double th0_err, double mgl_err, unsigned long long seed){
    unsigned long long rs = seed*6364136223846793005ULL + 1442695040888963407ULL;
    if (rs==0) rs=88172645463325252ULL;
    (void)xs(&rs);
    double bv_true = 0.02 + 0.18*urand(&rs);
    double J       = 0.05 + 0.25*urand(&rs);
    double mgl_true= 5.0 + 15.0*urand(&rs);
    double th0_true= -0.6 + 1.2*urand(&rs);
    double fc_true = 1.5 + 1.5*urand(&rs);
    double tau_loop= (0.3 + 1.2*urand(&rs))*1e-3;
    double sigma_v = 0.06*urand(&rs);
    double vq      = 1.745e-5*(1.0+9.0*urand(&rs));
    double th0_i   = -0.5 + 1.0*urand(&rs);
    double w0_i    = -0.5 + 1.0*urand(&rs);

    cp.mgl   = mgl_true*(1.0 - mgl_err + 2.0*mgl_err*urand(&rs));
    cp.theta0= th0_true + th0_err*(2.0*urand(&rs)-1.0);
    cp.fc    = fc_true; cp.bv = bv_true;
    cp.k_fric= k_fric; cp.G = G; cp.w0 = w0;
    cp.tlimit=6.0; cp.ff_limit=17.0; cp.total_limit=17.0;
    cp.deadzone=0.15;
    cp.flpf  = 0.15+0.5*urand(&rs);
    cp.alpf  = 0.05+0.3*urand(&rs);
    cp.hp    = 0.002+0.03*urand(&rs);
    cp.J_hat = J*(0.6+0.8*urand(&rs));

    int human_mode; double amp=0, t_on=0, t_off=1, freq=1;
    if (urand(&rs) < 0.20) human_mode=4;
    else {
        human_mode=(int)(urand(&rs)*4);
        amp=(2.0*urand(&rs)-1.0)*(5.0+25.0*urand(&rs));
        t_on=0.2*urand(&rs); t_off=t_on+0.3+1.2*urand(&rs); freq=0.5+2.0*urand(&rs);
    }

    Cs s; s.z1=th0_i; s.z2=w0_i; s.z3=0; s.fric=0; s.assist=0; s.hp_s=0; s.tau_prev=0;
    double theta=th0_i, omega=w0_i, tau_act=0, th_muscle=0;
    int dly=(int)delay_ms; if(dly<0)dly=0; if(dly>64)dly=64;
    double buf_v[65], buf_th[65];
    for(int i=0;i<=dly;i++){ buf_v[i]=0.0; buf_th[i]=th0_i; }
    int head=0;
    int flags=0, clamp_hit=0;
    for(int k=0;k<NSTEP;k++){
        double t=k*DT;
        double tt=0.0;
        switch(human_mode){
            case 0: tt=(t>=t_on&&t<t_off)?amp:0.0; break;
            case 1: tt=(t>=t_on&&t<t_off)?amp*(t-t_on)/(t_off-t_on):0.0; break;
            case 2: tt=(t>=t_on&&t<t_off)?amp*sin(6.283185307179586*freq*(t-t_on)):0.0; break;
            case 3: tt=((int)(t*4)%2==0&&t>=t_on&&t<t_off)?amp:0.0; break;
            default: tt=0.0;
        }
        th_muscle += (tt-th_muscle)*(DT/0.15);
        double th=th_muscle;
        double v_q=round(omega/vq)*vq;
        double v_noisy=v_q + sigma_v*nrand(&rs);
        double th_q=round(theta/(vq*DT))*(vq*DT);
        double v_fb=buf_v[head], th_fb=buf_th[head];
        buf_v[head]=v_noisy; buf_th[head]=th_q;
        head++; if(head>dly) head=0;
        double tau_cmd=ctrl(&cp,&s,th_fb,v_fb);
        double a_loop=exp(-DT/tau_loop);
        tau_act = tau_cmd + (tau_act-tau_cmd)*a_loop;
        double coul=(omega>1e-6)?fc_true:(omega<-1e-6?-fc_true:0.0);
        double acc=(tau_act + th - bv_true*omega - coul - mgl_true*cos(theta-th0_true))/J;
        omega+=acc*DT; theta+=omega*DT;
        if(!isfinite(theta)||!isfinite(omega)||!isfinite(tau_cmd)){ flags|=1; break; }
        if(fabs(tau_cmd)>cp.total_limit+1e-6) clamp_hit=1;
    }
    if(flags&1) return 1;
    if(fabs(omega)>60.0) flags|=2;
    if(human_mode==4){
        if(fabs(omega)>0.5) flags|=2;
        if(fabs(theta-th0_i)>0.30) flags|=4;
    }
    if(clamp_hit) flags|=8;
    (void)G;(void)k_fric;
    return flags?flags:0;
}

__global__ void kern(int mode, int n1, int n2, int per_cell,
                     double a_min,double a_max,double b_min,double b_max,
                     unsigned long long* cell_fail, unsigned long long* cell_nan)
{
    long long tid = (long long)blockIdx.x*blockDim.x + threadIdx.x;
    long long total = (long long)n1*n2*per_cell;
    if (tid>=total) return;
    int cell = (int)(tid / per_cell);
    int i1 = cell / n2, i2 = cell % n2;
    double p1 = a_min + (a_max-a_min)*(n1>1? (double)i1/(n1-1):0.0);
    double p2 = b_min + (b_max-b_min)*(n2>1? (double)i2/(n2-1):0.0);
    double G=0.5,k_fric=0.4,delay=2.0,w0=60.0,th0e=0.02,mgle=0.05;
    switch(mode){
        case 0: G=p1; k_fric=p2; break;
        case 1: delay=p1; w0=p2; break;
        case 2: th0e=p1; mgle=p2; break;
    }
    Cp cp;   // 其余字段由 run_trial 覆盖
    int f = run_trial(cp, G, k_fric, delay, w0, th0e, mgle, (unsigned long long)(tid+1));
    if (f) atomicAdd(&cell_fail[cell], 1ULL);
    if (f&1) atomicAdd(&cell_nan[cell], 1ULL);
}

static void dump(const char*fn, const char*name1,const char*name2,
                 int n1,int n2,double a0,double a1,double b0,double b1,
                 unsigned long long*fail,unsigned long long*nan,int per_cell)
{
    FILE*f=fopen(fn,"w");
    fprintf(f,"# %s(x) × %s(y), 每格 %d 次\n",name1,name2,per_cell);
    fprintf(f,"%s\\%s",name1,name2);
    for(int j=0;j<n2;j++) fprintf(f,",%.3f",b0+(b1-b0)*(n2>1?(double)j/(n2-1):0.0));
    fprintf(f,"\n");
    for(int i=0;i<n1;i++){
        fprintf(f,"%.3f",a0+(a1-a0)*(n1>1?(double)i/(n1-1):0.0));
        for(int j=0;j<n2;j++){
            int c=i*n2+j;
            fprintf(f,",%.2f",100.0*(double)fail[c]/per_cell);
        }
        fprintf(f,"\n");
    }
    fclose(f);
    // 汇总
    unsigned long long tot_fail=0, tot_nan=0, tot=0;
    for(int c=0;c<n1*n2;c++){ tot_fail+=fail[c]; tot_nan+=nan[c]; tot+=per_cell; }
    printf("  %s: 失败率 %.3f%% (NaN %.3f%%), 总 %llu 次\n", fn,
           100.0*(double)tot_fail/tot, 100.0*(double)tot_nan/tot, tot);
}

int main(int argc,char**argv){
    int n1 = (argc>1)?atoi(argv[1]):40;      // x 轴格数
    int n2 = (argc>2)?atoi(argv[2]):40;      // y 轴格数
    int per= (argc>3)?atoi(argv[3]):625;     // 每格次数 (n1*n2*per = 总次数)
    long long total=(long long)n1*n2*per;
    printf("P100 验证: %d×%d 格 × %d 次/格 = %lld 次/热图\n", n1,n2,per,total);

    unsigned long long *dfail,*dnan;
    cudaMalloc(&dfail,sizeof(unsigned long long)*n1*n2);
    cudaMalloc(&dnan ,sizeof(unsigned long long)*n1*n2);
    int threads=256, blocks=(int)((total+threads-1)/threads);

    // 三种热图
    struct { const char* fn; int mode; const char* x; const char* y; double a0,a1,b0,b1; } H[3]={
      {"heat_G_kfric.csv", 0,"G_assist","k_fric",     0.0, 1.5, 0.0, 1.3},
      {"heat_delay_w0.csv",1,"delay_ms","ESO_w0",     0.0,12.0, 10.0,200.0},
      {"heat_calib.csv",   2,"th0_err_rad","mgl_err", 0.0, 0.30, 0.0, 0.30}
    };
    for(int m=0;m<3;m++){
        cudaMemset(dfail,0,sizeof(unsigned long long)*n1*n2);
        cudaMemset(dnan ,0,sizeof(unsigned long long)*n1*n2);
        cudaEvent_t e0,e1; cudaEventCreate(&e0); cudaEventCreate(&e1);
        cudaEventRecord(e0);
        kern<<<blocks,threads>>>(H[m].mode,n1,n2,per,H[m].a0,H[m].a1,H[m].b0,H[m].b1,dfail,dnan);
        cudaDeviceSynchronize();
        cudaEventRecord(e1); cudaEventSynchronize(e1);
        float ms=0; cudaEventElapsedTime(&ms,e0,e1);
        unsigned long long *hf=new unsigned long long[n1*n2], *hn=new unsigned long long[n1*n2];
        cudaMemcpy(hf,dfail,sizeof(unsigned long long)*n1*n2,cudaMemcpyDeviceToHost);
        cudaMemcpy(hn,dnan ,sizeof(unsigned long long)*n1*n2,cudaMemcpyDeviceToHost);
        printf("  [mode %d] %s 用时 %.1f ms (%.0f 次/s)\n", m, H[m].fn, ms, (double)total/(ms/1000.0));
        dump(H[m].fn,H[m].x,H[m].y,n1,n2,H[m].a0,H[m].a1,H[m].b0,H[m].b1,hf,hn,per);
        delete[] hf; delete[] hn;
    }
    cudaFree(dfail); cudaFree(dnan);
    return 0;
}
