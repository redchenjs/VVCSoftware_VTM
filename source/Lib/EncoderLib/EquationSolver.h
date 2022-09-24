/*
 * EquationSolver.h
 *
 *  Created on: 2022-07-30 18:20
 *      Author: Jack Chen <redchenjs@live.com>
 */

#ifndef __EQUATIONSOLVER__
#define __EQUATIONSOLVER__

#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>

#define DEBUG

class EquationSolver
{
private:
    double C[7][7] = { 0.0 };

    void print_mat(const char *str, int n, double C[7][7]);
    void print_mat(char idx, int k, int n, double C[7][7]);
    void print_mat(char idx, int k, int m, int n, double C[7][7]);

    void print_res(int n, double C[7][7]);
    void print_res(int n, double C[7][7], int scale);

public:
    void load_data(const int64_t i64EqualCoeff[7][7], int iParaNum, int scale);
    void save_data(double dAffinePara[6], int iParaNum, int scale);

    void method_gja(int n);
    void method_dfa(int n, int scale);
    void method_dfa2(int n, int scale);
    void method_dfa2s(int n, int scale);
    void method_dfa3(int n, int scale);
    void method_dfa3s(int n, int scale);
    void method_cra(int n, int scale);
};

#endif // __EQUATIONSOLVER__
