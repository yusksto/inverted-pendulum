#include <iostream>
#include <cmath>
#include <cstdio>
#include <cassert>
#include <omp.h>


double RungeKutta_1(double, double, double, double);
double RungeKutta_2(double, double, double, double);
void RungeKutta_3(double, double, double&, double&, double, double);

const double dt = 0.01;
const double t_0 = 0;
const double t_1 = 2;
const double g = 9.8;

const double m = 0.1;
const double R = 0.025;
const double I_G = 0.0001;
const double I = I_G + m * R * R;

const double p = sqrt(I * I - m * g * R * I);
const double p_2 = 10;
const double p_1 = g + m * R / 4 / I * p_2 * p_2;

const double p_4 = 120;
const double p_3 = 0;

const double p_5 = m * R * (g - p_1) / I;
const double p_6 = -m * R * p_2 / I;
const double p_7 = -m * R * p_3 / I;
const double p_8 = -m * R * p_4 / I;



const double θ_0_0 = 0.1;
const double θ_1_0 = 0;
const double x_0 = 0;
const double v_x_0 = 0.01;

const double range = R * 2;


int main()
{
    //gnuplot設定
    FILE* gp = _popen("gnuplot", "w");
    assert(gp);
    fprintf(gp, "reset\n");
    fprintf(gp, "set terminal gif animate optimize delay %d size %d,%d\n", (int)(5), (int)(1080), (int)(1080));
    fprintf(gp, "set output 'tmp.gif'\n");
    fprintf(gp, "unset key\n");
    fprintf(gp, "set yr [%lf:%lf]\n", -R, range * 1.5);
    fprintf(gp, "set xr [%lf:%lf]\n", -R * 2, R * 2);
    fprintf(gp, "set xlabel 'x[m]'\n");
    fprintf(gp, "set ylabel 'y[m]'\n");
    fprintf(gp, "set size square\n");

    double θ_0[2] = { θ_0_0,θ_0_0 };
    double θ_1[2] = { θ_1_0, θ_1_0 };
    double x[2] = { x_0, x_0 };
    double v_x[2] = { v_x_0, v_x_0 };

    std::cout << p_1 << " " << p_2 << " " << p_3 << " " << p_4 << std::endl;

    int a = 1;

    //開始θ=0.1
    //v_x=0へ
    for (double t = t_0; t < t_1; t += dt)
    {
        if (a == 1)
        {
            fprintf(gp, "set title 'x-y[m] t = %lf[s] θ=%lf[rad] x=%lf[m] v=%lf[m/s]'\n", t, θ_0[0], x[0], v_x[0]);
            fprintf(gp, "plot '-' pt 7 ps 2 lc \'dark-magenta\', '-' with lines lc \'dark-magenta\'\n");
            fprintf(gp, "%lf, %lf\n", R * sin(θ_0[0]) + x[0], R * cos(θ_0[0]));
            fprintf(gp, "%lf, %lf\n", x[0], 0);
            fprintf(gp, "e\n");
            fprintf(gp, "%lf, %lf\n", R * sin(θ_0[0]) + x[0], R * cos(θ_0[0]));
            fprintf(gp, "%lf, %lf\n", x[0], 0);
            fprintf(gp, "e\n");
            fflush(gp);
            std::cout << t << std::endl;
            a = 0;
        }
        RungeKutta_3(θ_0[0], θ_1[0], θ_0[1], θ_1[1], v_x[0], x[0]);

        v_x[1] += dt * (p_1 * (θ_0[0] + θ_0[1]) + p_2 * (θ_1[0] + θ_1[1]) + p_3 * (x[0] + x[1]) + p_4 * (v_x[0] + v_x[1])) / 2;
        x[1] += dt * (v_x[0] + v_x[1]) / 2;

        θ_0[0] = θ_0[1];
        θ_1[0] = θ_1[1];
        x[0] = x[1];
        v_x[0] = v_x[1];        

        a++;
    }



    fprintf(gp, "set out\n");
    _pclose(gp);
    return 0;
}

double RungeKutta_1(double θ_0, double θ_1, double v_x, double x) {
    return θ_0 * p_5 + θ_1 * p_6 + v_x * p_8 + x * p_7;
}

double RungeKutta_2(double θ_0, double θ_1, double v_x, double x) {
    double j_1 = dt * RungeKutta_1(θ_0, θ_1, v_x, x);
    double k_1 = dt * θ_1;
    double j_2 = dt * RungeKutta_1(θ_0 + k_1 / 2, θ_1 + j_1 / 2, v_x, x);
    double k_2 = dt * (θ_1 + j_1 / 2);
    double j_3 = dt * RungeKutta_1(θ_0 + k_2 / 2, θ_1 + j_2 / 2, v_x, x);
    double k_3 = dt * (θ_1 + j_2 / 2);
    double j_4 = dt * RungeKutta_1(θ_0 + k_3, θ_1 + j_3, v_x, x);
    double k_4 = dt * (θ_1 + j_2);
    return θ_1 + (j_1 + 2 * j_2 + 2 * j_3 + j_4) / 6;
}

void RungeKutta_3(double θ_0, double θ_1, double& θ_0_, double& θ_1_, double v_x, double x) {
    double j_1 = dt * RungeKutta_1(θ_0, θ_1, v_x, x);
    double k_1 = dt * RungeKutta_2(θ_0, θ_1, v_x, x);
    double j_2 = dt * RungeKutta_1(θ_0 + k_1 / 2, θ_1 + j_1 / 2, v_x, x);
    double k_2 = dt * (RungeKutta_2(θ_0 + k_1 / 2, θ_1 + j_1 / 2, v_x, x) + j_1 / 2);
    double j_3 = dt * RungeKutta_1(θ_0 + k_2 / 2, θ_1 + j_2 / 2, v_x, x);
    double k_3 = dt * (RungeKutta_2(θ_0 + k_2 / 2, θ_1 + j_2 / 2, v_x, x) + j_2 / 2);
    double j_4 = dt * RungeKutta_1(θ_0 + k_3, θ_1 + j_3, v_x, x);
    double k_4 = dt * (RungeKutta_2(θ_0 + k_3, θ_1 + j_3, v_x, x) + j_2);
    θ_1_ = θ_1 + (j_1 + 2 * j_2 + 2 * j_3 + j_4) / 6;
    θ_0_ = θ_0 + (k_1 + 2 * k_2 + 2 * k_3 + k_4) / 6;
}
