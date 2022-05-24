#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_106967697070233423);
void live_err_fun(double *nom_x, double *delta_x, double *out_1422853424452190626);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_6858336240497983502);
void live_H_mod_fun(double *state, double *out_8417267415678021814);
void live_f_fun(double *state, double dt, double *out_1910842593480790213);
void live_F_fun(double *state, double dt, double *out_101320918171338845);
void live_h_4(double *state, double *unused, double *out_2837472755272767260);
void live_H_4(double *state, double *unused, double *out_4451958455480422932);
void live_h_9(double *state, double *unused, double *out_6663834342115976657);
void live_H_9(double *state, double *unused, double *out_2835260479784024538);
void live_h_10(double *state, double *unused, double *out_3099628852342878157);
void live_H_10(double *state, double *unused, double *out_7098970975798624777);
void live_h_12(double *state, double *unused, double *out_1882254676201215969);
void live_H_12(double *state, double *unused, double *out_7613527241186395688);
void live_h_31(double *state, double *unused, double *out_475750438276490537);
void live_H_31(double *state, double *unused, double *out_3313060984876552572);
void live_h_32(double *state, double *unused, double *out_242924299372773870);
void live_H_32(double *state, double *unused, double *out_954131613167075087);
void live_h_13(double *state, double *unused, double *out_4138566346372002600);
void live_H_13(double *state, double *unused, double *out_8865999913403457686);
void live_h_14(double *state, double *unused, double *out_6663834342115976657);
void live_H_14(double *state, double *unused, double *out_2835260479784024538);
void live_h_33(double *state, double *unused, double *out_4018036290532756885);
void live_H_33(double *state, double *unused, double *out_9111289895165898873);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}