#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with sympy 1.9                        *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_2732827909754311456) {
   out_2732827909754311456[0] = delta_x[0] + nom_x[0];
   out_2732827909754311456[1] = delta_x[1] + nom_x[1];
   out_2732827909754311456[2] = delta_x[2] + nom_x[2];
   out_2732827909754311456[3] = delta_x[3] + nom_x[3];
   out_2732827909754311456[4] = delta_x[4] + nom_x[4];
   out_2732827909754311456[5] = delta_x[5] + nom_x[5];
   out_2732827909754311456[6] = delta_x[6] + nom_x[6];
   out_2732827909754311456[7] = delta_x[7] + nom_x[7];
   out_2732827909754311456[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8245636438625739729) {
   out_8245636438625739729[0] = -nom_x[0] + true_x[0];
   out_8245636438625739729[1] = -nom_x[1] + true_x[1];
   out_8245636438625739729[2] = -nom_x[2] + true_x[2];
   out_8245636438625739729[3] = -nom_x[3] + true_x[3];
   out_8245636438625739729[4] = -nom_x[4] + true_x[4];
   out_8245636438625739729[5] = -nom_x[5] + true_x[5];
   out_8245636438625739729[6] = -nom_x[6] + true_x[6];
   out_8245636438625739729[7] = -nom_x[7] + true_x[7];
   out_8245636438625739729[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_7768268916690382821) {
   out_7768268916690382821[0] = 1.0;
   out_7768268916690382821[1] = 0;
   out_7768268916690382821[2] = 0;
   out_7768268916690382821[3] = 0;
   out_7768268916690382821[4] = 0;
   out_7768268916690382821[5] = 0;
   out_7768268916690382821[6] = 0;
   out_7768268916690382821[7] = 0;
   out_7768268916690382821[8] = 0;
   out_7768268916690382821[9] = 0;
   out_7768268916690382821[10] = 1.0;
   out_7768268916690382821[11] = 0;
   out_7768268916690382821[12] = 0;
   out_7768268916690382821[13] = 0;
   out_7768268916690382821[14] = 0;
   out_7768268916690382821[15] = 0;
   out_7768268916690382821[16] = 0;
   out_7768268916690382821[17] = 0;
   out_7768268916690382821[18] = 0;
   out_7768268916690382821[19] = 0;
   out_7768268916690382821[20] = 1.0;
   out_7768268916690382821[21] = 0;
   out_7768268916690382821[22] = 0;
   out_7768268916690382821[23] = 0;
   out_7768268916690382821[24] = 0;
   out_7768268916690382821[25] = 0;
   out_7768268916690382821[26] = 0;
   out_7768268916690382821[27] = 0;
   out_7768268916690382821[28] = 0;
   out_7768268916690382821[29] = 0;
   out_7768268916690382821[30] = 1.0;
   out_7768268916690382821[31] = 0;
   out_7768268916690382821[32] = 0;
   out_7768268916690382821[33] = 0;
   out_7768268916690382821[34] = 0;
   out_7768268916690382821[35] = 0;
   out_7768268916690382821[36] = 0;
   out_7768268916690382821[37] = 0;
   out_7768268916690382821[38] = 0;
   out_7768268916690382821[39] = 0;
   out_7768268916690382821[40] = 1.0;
   out_7768268916690382821[41] = 0;
   out_7768268916690382821[42] = 0;
   out_7768268916690382821[43] = 0;
   out_7768268916690382821[44] = 0;
   out_7768268916690382821[45] = 0;
   out_7768268916690382821[46] = 0;
   out_7768268916690382821[47] = 0;
   out_7768268916690382821[48] = 0;
   out_7768268916690382821[49] = 0;
   out_7768268916690382821[50] = 1.0;
   out_7768268916690382821[51] = 0;
   out_7768268916690382821[52] = 0;
   out_7768268916690382821[53] = 0;
   out_7768268916690382821[54] = 0;
   out_7768268916690382821[55] = 0;
   out_7768268916690382821[56] = 0;
   out_7768268916690382821[57] = 0;
   out_7768268916690382821[58] = 0;
   out_7768268916690382821[59] = 0;
   out_7768268916690382821[60] = 1.0;
   out_7768268916690382821[61] = 0;
   out_7768268916690382821[62] = 0;
   out_7768268916690382821[63] = 0;
   out_7768268916690382821[64] = 0;
   out_7768268916690382821[65] = 0;
   out_7768268916690382821[66] = 0;
   out_7768268916690382821[67] = 0;
   out_7768268916690382821[68] = 0;
   out_7768268916690382821[69] = 0;
   out_7768268916690382821[70] = 1.0;
   out_7768268916690382821[71] = 0;
   out_7768268916690382821[72] = 0;
   out_7768268916690382821[73] = 0;
   out_7768268916690382821[74] = 0;
   out_7768268916690382821[75] = 0;
   out_7768268916690382821[76] = 0;
   out_7768268916690382821[77] = 0;
   out_7768268916690382821[78] = 0;
   out_7768268916690382821[79] = 0;
   out_7768268916690382821[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_575843426271879826) {
   out_575843426271879826[0] = state[0];
   out_575843426271879826[1] = state[1];
   out_575843426271879826[2] = state[2];
   out_575843426271879826[3] = state[3];
   out_575843426271879826[4] = state[4];
   out_575843426271879826[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_575843426271879826[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_575843426271879826[7] = state[7];
   out_575843426271879826[8] = state[8];
}
void F_fun(double *state, double dt, double *out_2836379347880580508) {
   out_2836379347880580508[0] = 1;
   out_2836379347880580508[1] = 0;
   out_2836379347880580508[2] = 0;
   out_2836379347880580508[3] = 0;
   out_2836379347880580508[4] = 0;
   out_2836379347880580508[5] = 0;
   out_2836379347880580508[6] = 0;
   out_2836379347880580508[7] = 0;
   out_2836379347880580508[8] = 0;
   out_2836379347880580508[9] = 0;
   out_2836379347880580508[10] = 1;
   out_2836379347880580508[11] = 0;
   out_2836379347880580508[12] = 0;
   out_2836379347880580508[13] = 0;
   out_2836379347880580508[14] = 0;
   out_2836379347880580508[15] = 0;
   out_2836379347880580508[16] = 0;
   out_2836379347880580508[17] = 0;
   out_2836379347880580508[18] = 0;
   out_2836379347880580508[19] = 0;
   out_2836379347880580508[20] = 1;
   out_2836379347880580508[21] = 0;
   out_2836379347880580508[22] = 0;
   out_2836379347880580508[23] = 0;
   out_2836379347880580508[24] = 0;
   out_2836379347880580508[25] = 0;
   out_2836379347880580508[26] = 0;
   out_2836379347880580508[27] = 0;
   out_2836379347880580508[28] = 0;
   out_2836379347880580508[29] = 0;
   out_2836379347880580508[30] = 1;
   out_2836379347880580508[31] = 0;
   out_2836379347880580508[32] = 0;
   out_2836379347880580508[33] = 0;
   out_2836379347880580508[34] = 0;
   out_2836379347880580508[35] = 0;
   out_2836379347880580508[36] = 0;
   out_2836379347880580508[37] = 0;
   out_2836379347880580508[38] = 0;
   out_2836379347880580508[39] = 0;
   out_2836379347880580508[40] = 1;
   out_2836379347880580508[41] = 0;
   out_2836379347880580508[42] = 0;
   out_2836379347880580508[43] = 0;
   out_2836379347880580508[44] = 0;
   out_2836379347880580508[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_2836379347880580508[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_2836379347880580508[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2836379347880580508[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2836379347880580508[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_2836379347880580508[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_2836379347880580508[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_2836379347880580508[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_2836379347880580508[53] = -9.8000000000000007*dt;
   out_2836379347880580508[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_2836379347880580508[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_2836379347880580508[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2836379347880580508[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2836379347880580508[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_2836379347880580508[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_2836379347880580508[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_2836379347880580508[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2836379347880580508[62] = 0;
   out_2836379347880580508[63] = 0;
   out_2836379347880580508[64] = 0;
   out_2836379347880580508[65] = 0;
   out_2836379347880580508[66] = 0;
   out_2836379347880580508[67] = 0;
   out_2836379347880580508[68] = 0;
   out_2836379347880580508[69] = 0;
   out_2836379347880580508[70] = 1;
   out_2836379347880580508[71] = 0;
   out_2836379347880580508[72] = 0;
   out_2836379347880580508[73] = 0;
   out_2836379347880580508[74] = 0;
   out_2836379347880580508[75] = 0;
   out_2836379347880580508[76] = 0;
   out_2836379347880580508[77] = 0;
   out_2836379347880580508[78] = 0;
   out_2836379347880580508[79] = 0;
   out_2836379347880580508[80] = 1;
}
void h_25(double *state, double *unused, double *out_7148684108209273997) {
   out_7148684108209273997[0] = state[6];
}
void H_25(double *state, double *unused, double *out_7474152819984158976) {
   out_7474152819984158976[0] = 0;
   out_7474152819984158976[1] = 0;
   out_7474152819984158976[2] = 0;
   out_7474152819984158976[3] = 0;
   out_7474152819984158976[4] = 0;
   out_7474152819984158976[5] = 0;
   out_7474152819984158976[6] = 1;
   out_7474152819984158976[7] = 0;
   out_7474152819984158976[8] = 0;
}
void h_24(double *state, double *unused, double *out_7163463865602985993) {
   out_7163463865602985993[0] = state[4];
   out_7163463865602985993[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1346932317822476779) {
   out_1346932317822476779[0] = 0;
   out_1346932317822476779[1] = 0;
   out_1346932317822476779[2] = 0;
   out_1346932317822476779[3] = 0;
   out_1346932317822476779[4] = 1;
   out_1346932317822476779[5] = 0;
   out_1346932317822476779[6] = 0;
   out_1346932317822476779[7] = 0;
   out_1346932317822476779[8] = 0;
   out_1346932317822476779[9] = 0;
   out_1346932317822476779[10] = 0;
   out_1346932317822476779[11] = 0;
   out_1346932317822476779[12] = 0;
   out_1346932317822476779[13] = 0;
   out_1346932317822476779[14] = 1;
   out_1346932317822476779[15] = 0;
   out_1346932317822476779[16] = 0;
   out_1346932317822476779[17] = 0;
}
void h_30(double *state, double *unused, double *out_6873490045924768108) {
   out_6873490045924768108[0] = state[4];
}
void H_30(double *state, double *unused, double *out_8454258295218144013) {
   out_8454258295218144013[0] = 0;
   out_8454258295218144013[1] = 0;
   out_8454258295218144013[2] = 0;
   out_8454258295218144013[3] = 0;
   out_8454258295218144013[4] = 1;
   out_8454258295218144013[5] = 0;
   out_8454258295218144013[6] = 0;
   out_8454258295218144013[7] = 0;
   out_8454258295218144013[8] = 0;
}
void h_26(double *state, double *unused, double *out_1388406014583935653) {
   out_1388406014583935653[0] = state[7];
}
void H_26(double *state, double *unused, double *out_7668065283964592039) {
   out_7668065283964592039[0] = 0;
   out_7668065283964592039[1] = 0;
   out_7668065283964592039[2] = 0;
   out_7668065283964592039[3] = 0;
   out_7668065283964592039[4] = 0;
   out_7668065283964592039[5] = 0;
   out_7668065283964592039[6] = 0;
   out_7668065283964592039[7] = 1;
   out_7668065283964592039[8] = 0;
}
void h_27(double *state, double *unused, double *out_8693654644691579968) {
   out_8693654644691579968[0] = state[3];
}
void H_27(double *state, double *unused, double *out_7817722466690982692) {
   out_7817722466690982692[0] = 0;
   out_7817722466690982692[1] = 0;
   out_7817722466690982692[2] = 0;
   out_7817722466690982692[3] = 1;
   out_7817722466690982692[4] = 0;
   out_7817722466690982692[5] = 0;
   out_7817722466690982692[6] = 0;
   out_7817722466690982692[7] = 0;
   out_7817722466690982692[8] = 0;
}
void h_29(double *state, double *unused, double *out_8536467976340450823) {
   out_8536467976340450823[0] = state[1];
}
void H_29(double *state, double *unused, double *out_7944026950903751829) {
   out_7944026950903751829[0] = 0;
   out_7944026950903751829[1] = 1;
   out_7944026950903751829[2] = 0;
   out_7944026950903751829[3] = 0;
   out_7944026950903751829[4] = 0;
   out_7944026950903751829[5] = 0;
   out_7944026950903751829[6] = 0;
   out_7944026950903751829[7] = 0;
   out_7944026950903751829[8] = 0;
}
void h_28(double *state, double *unused, double *out_7750144791789917533) {
   out_7750144791789917533[0] = state[0];
}
void H_28(double *state, double *unused, double *out_5420318105736269213) {
   out_5420318105736269213[0] = 1;
   out_5420318105736269213[1] = 0;
   out_5420318105736269213[2] = 0;
   out_5420318105736269213[3] = 0;
   out_5420318105736269213[4] = 0;
   out_5420318105736269213[5] = 0;
   out_5420318105736269213[6] = 0;
   out_5420318105736269213[7] = 0;
   out_5420318105736269213[8] = 0;
}
void h_31(double *state, double *unused, double *out_5142612769659617490) {
   out_5142612769659617490[0] = state[8];
}
void H_31(double *state, double *unused, double *out_8294273386197943515) {
   out_8294273386197943515[0] = 0;
   out_8294273386197943515[1] = 0;
   out_8294273386197943515[2] = 0;
   out_8294273386197943515[3] = 0;
   out_8294273386197943515[4] = 0;
   out_8294273386197943515[5] = 0;
   out_8294273386197943515[6] = 0;
   out_8294273386197943515[7] = 0;
   out_8294273386197943515[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_2732827909754311456) {
  err_fun(nom_x, delta_x, out_2732827909754311456);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8245636438625739729) {
  inv_err_fun(nom_x, true_x, out_8245636438625739729);
}
void car_H_mod_fun(double *state, double *out_7768268916690382821) {
  H_mod_fun(state, out_7768268916690382821);
}
void car_f_fun(double *state, double dt, double *out_575843426271879826) {
  f_fun(state,  dt, out_575843426271879826);
}
void car_F_fun(double *state, double dt, double *out_2836379347880580508) {
  F_fun(state,  dt, out_2836379347880580508);
}
void car_h_25(double *state, double *unused, double *out_7148684108209273997) {
  h_25(state, unused, out_7148684108209273997);
}
void car_H_25(double *state, double *unused, double *out_7474152819984158976) {
  H_25(state, unused, out_7474152819984158976);
}
void car_h_24(double *state, double *unused, double *out_7163463865602985993) {
  h_24(state, unused, out_7163463865602985993);
}
void car_H_24(double *state, double *unused, double *out_1346932317822476779) {
  H_24(state, unused, out_1346932317822476779);
}
void car_h_30(double *state, double *unused, double *out_6873490045924768108) {
  h_30(state, unused, out_6873490045924768108);
}
void car_H_30(double *state, double *unused, double *out_8454258295218144013) {
  H_30(state, unused, out_8454258295218144013);
}
void car_h_26(double *state, double *unused, double *out_1388406014583935653) {
  h_26(state, unused, out_1388406014583935653);
}
void car_H_26(double *state, double *unused, double *out_7668065283964592039) {
  H_26(state, unused, out_7668065283964592039);
}
void car_h_27(double *state, double *unused, double *out_8693654644691579968) {
  h_27(state, unused, out_8693654644691579968);
}
void car_H_27(double *state, double *unused, double *out_7817722466690982692) {
  H_27(state, unused, out_7817722466690982692);
}
void car_h_29(double *state, double *unused, double *out_8536467976340450823) {
  h_29(state, unused, out_8536467976340450823);
}
void car_H_29(double *state, double *unused, double *out_7944026950903751829) {
  H_29(state, unused, out_7944026950903751829);
}
void car_h_28(double *state, double *unused, double *out_7750144791789917533) {
  h_28(state, unused, out_7750144791789917533);
}
void car_H_28(double *state, double *unused, double *out_5420318105736269213) {
  H_28(state, unused, out_5420318105736269213);
}
void car_h_31(double *state, double *unused, double *out_5142612769659617490) {
  h_31(state, unused, out_5142612769659617490);
}
void car_H_31(double *state, double *unused, double *out_8294273386197943515) {
  H_31(state, unused, out_8294273386197943515);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_init(car);
