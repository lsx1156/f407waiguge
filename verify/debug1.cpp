// debug1.cpp — 单轨迹 dump: 完美标定, G=0, 无摩擦补偿, 无延迟/噪声
// 期望: 臂保持初始位置不动(|ω|→0). 若动/发散, 从这里看机理.
#include <cstdio>
#include <cmath>
#include <cstring>
static const double DT = 1e-3;
static inline double clampd(double v,double lo,double hi){return v<lo?lo:(v>hi?hi:v);}

struct Cp { double mgl,theta0,fc,bv,k_fric,G,w0,tlimit,deadzone,flpf,alpf,hp,J_hat; };
struct Cs { double z1,z2,z3,vel_prev,fric,assist,hp_s,tau_prev; };

static double ctrl(const Cp*p, Cs*s, double theta, double omega){
    double w=omega, sgn=(w>1e-4)?1.0:(w<-1e-4?-1.0:0.0);
    double fr=p->k_fric*(p->fc*sgn + p->bv*w);
    if(fabs(w)<p->deadzone) fr=0.0;
    s->fric += p->flpf*(fr - s->fric);
    double tg = p->mgl*cos(theta - p->theta0);
    double b0=1.0/p->J_hat;
    double e=s->z1-theta;
    double b1=3*p->w0,b2=3*p->w0*p->w0,b3=p->w0*p->w0*p->w0;
    s->z1+=DT*(s->z2-b1*e);
    s->z2+=DT*(s->z3+b0*s->tau_prev-b2*e);
    s->z3+=DT*(-b3*e);
    double te=p->J_hat*s->z3;
    s->hp_s += p->hp*(te - s->hp_s);
    double thd = te - s->hp_s;
    s->assist += p->alpf*(p->G*thd - s->assist);
    double tau = tg + s->fric + s->assist;
    double out = clampd(tau,-p->tlimit,p->tlimit);
    s->tau_prev=out;
    return out;
}

int main(){
    Cp p; Cs s; memset(&p,0,sizeof(p)); memset(&s,0,sizeof(s));
    p.mgl=12.6; p.theta0=0.0; p.fc=2.0; p.bv=0.1; p.k_fric=0.0;
    p.G=0.0; p.w0=50.0; p.tlimit=6.0; p.deadzone=0.15; p.flpf=0.3; p.alpf=0.15; p.hp=0.01; p.J_hat=0.2;
    double J=0.2, mgl=12.6, th0=0.0, fc=2.0, bv=0.1, tau_loop=0.5e-3;
    double theta=0.30, omega=0.0, tau_act=0.0;
    s.z1=theta; s.z2=omega;
    printf("t(s)    theta    omega    tau_cmd   tau_act   z3       tau_ext\n");
    for(int k=0;k<3000;k++){
        double t=k*DT;
        double tc=ctrl(&p,&s,theta,omega);
        double a=exp(-DT/tau_loop);
        tau_act = tc + (tau_act-tc)*a;
        double coul=(omega>1e-6)?fc:(omega<-1e-6?-fc:0.0);
        double acc=(tau_act - bv*omega - coul - mgl*cos(theta-th0))/J;
        omega+=acc*DT; theta+=omega*DT;
        if(k%100==0 || k<20)
            printf("%.3f  %8.4f %8.4f %8.3f %8.3f %8.3f %8.3f\n",
                   t,theta,omega,tc,tau_act,s.z3,p.J_hat*s.z3);
    }
    return 0;
}
