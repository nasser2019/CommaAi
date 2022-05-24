#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_2732827909754311456);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8245636438625739729);
void car_H_mod_fun(double *state, double *out_7768268916690382821);
void car_f_fun(double *state, double dt, double *out_575843426271879826);
void car_F_fun(double *state, double dt, double *out_2836379347880580508);
void car_h_25(double *state, double *unused, double *out_7148684108209273997);
void car_H_25(double *state, double *unused, double *out_7474152819984158976);
void car_h_24(double *state, double *unused, double *out_7163463865602985993);
void car_H_24(double *state, double *unused, double *out_1346932317822476779);
void car_h_30(double *state, double *unused, double *out_6873490045924768108);
void car_H_30(double *state, double *unused, double *out_8454258295218144013);
void car_h_26(double *state, double *unused, double *out_1388406014583935653);
void car_H_26(double *state, double *unused, double *out_7668065283964592039);
void car_h_27(double *state, double *unused, double *out_8693654644691579968);
void car_H_27(double *state, double *unused, double *out_7817722466690982692);
void car_h_29(double *state, double *unused, double *out_8536467976340450823);
void car_H_29(double *state, double *unused, double *out_7944026950903751829);
void car_h_28(double *state, double *unused, double *out_7750144791789917533);
void car_H_28(double *state, double *unused, double *out_5420318105736269213);
void car_h_31(double *state, double *unused, double *out_5142612769659617490);
void car_H_31(double *state, double *unused, double *out_8294273386197943515);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}