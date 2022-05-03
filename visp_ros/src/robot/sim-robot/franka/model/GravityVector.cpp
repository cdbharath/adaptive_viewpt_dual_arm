/*
 * GravityVector.cpp
 *
 *  Created on: Jun 15, 2021
 *      Author: Alexander Oliva
 *
 *  C. Gaz, M. Cognetti, A. Oliva, P. Robuffo Giordano, A. De Luca, 'Dynamic
 *  Identification of the Franka Emika Panda Robot With Retrieval of Feasible
 *  Parameters Using Penalty-Based Optimization'. IEEE RA-L, 2019.
 *
 *  ver: 2.0
 *   This version is parametrized w.r.t. both the gravitational acceleration
 *   vector g0 and the payload parameters.
 *    - flMcom is the Homogeneous matrix of the CoM in flange frame.
 *    - g0 must be expressed in floor frame: g0 = fRw * [0, 0, -9.80665]^T [m/s^2]
 *    - mL is the total mass of the payload.   [kg]
 *
 */

#include "franka_model.h"

namespace franka_model
{

vpColVector
gravityVector( const vpColVector &q, const double mL, const vpHomogeneousMatrix &flMcom, const vpColVector &g0 )
{

  vpColVector g( njoints, 0 );
  double clx, cly, clz, gx, gy, gz, cq1, cq2, cq3, cq4, cq5, cq6, cq7, sq1, sq2, sq3, sq4, sq5, sq6, sq7, cq12, cq13,
      cq14, cq15;

  clx = flMcom[0][3];
  cly = flMcom[1][3];
  clz = 0.107 + flMcom[2][3];

  gx = g0[0];
  gy = g0[1];
  gz = g0[2];

  cq1 = cos( q[0] );
  cq2 = cos( q[1] );
  cq3 = cos( q[2] );
  cq4 = cos( q[3] );
  cq5 = cos( q[4] );
  cq6 = cos( q[5] );
  cq7 = cos( q[6] );

  cq12 = cq1 * cq2;
  cq13 = cq1 * cq3;
  cq14 = cq1 * cq4;
  cq15 = cq1 * cq5;

  sq1 = sin( q[0] );
  sq2 = sin( q[1] );
  sq3 = sin( q[2] );
  sq4 = sin( q[3] );
  sq5 = sin( q[4] );
  sq6 = sin( q[5] );
  sq7 = sin( q[6] );

  g[0] =
      gx * cq1 * 1.2604999774E-2 - gy * cq1 * 1.92614005E-2 + gx * sq1 * 1.92614005E-2 + gy * sq1 * 1.2604999774E-2 +
      gx * cq13 * 2.8227094878E-2 + gy * cq12 * 2.031994566E-3 - gx * cq2 * sq1 * 2.031994566E-3 +
      gx * cq1 * sq3 * 6.23477394872E-1 - gy * cq1 * sq2 * 2.871915091512 + gy * cq3 * sq1 * 2.8227094878E-2 +
      gx * sq1 * sq2 * 2.871915091512 + gy * sq1 * sq3 * 6.23477394872E-1 + gx * cq13 * cq5 * 6.7870631425E-2 -
      gy * cq12 * cq3 * 6.23477394872E-1 + gx * cq2 * cq3 * sq1 * 6.23477394872E-1 - gx * cq14 * sq3 * 4.2939970965E-1 -
      gx * cq13 * sq5 * 1.4653732538E-2 + gy * cq12 * sq3 * 2.8227094878E-2 - gy * cq14 * sq2 * 1.438243105603 +
      gy * cq3 * cq5 * sq1 * 6.7870631425E-2 - gx * cq2 * sq1 * sq3 * 2.8227094878E-2 +
      gx * cq4 * sq1 * sq2 * 1.438243105603 - gx * cq1 * sq3 * sq4 * 1.438243105603 +
      gy * cq1 * sq2 * sq4 * 4.2939970965E-1 - gy * cq4 * sq1 * sq3 * 4.2939970965E-1 -
      gy * cq3 * sq1 * sq5 * 1.4653732538E-2 - gx * sq1 * sq2 * sq4 * 4.2939970965E-1 -
      gy * sq1 * sq3 * sq4 * 1.438243105603 + gx * cq1 * sq3 * ( mL + 7.35522E-1 ) * ( 0.0825 ) -
      gy * cq1 * sq2 * ( mL + 7.35522E-1 ) * ( 0.316 ) + gx * sq1 * sq2 * ( mL + 7.35522E-1 ) * ( 0.316 ) +
      gy * sq1 * sq3 * ( mL + 7.35522E-1 ) * ( 0.0825 ) + gy * cq12 * cq3 * cq4 * 4.2939970965E-1 -
      gx * cq2 * cq3 * cq4 * sq1 * 4.2939970965E-1 - gx * cq14 * cq5 * sq3 * 1.4653732538E-2 +
      gx * cq13 * cq6 * sq5 * 1.00241616695E-1 + gy * cq12 * cq3 * sq4 * 1.438243105603 +
      gy * cq12 * cq5 * sq3 * 6.7870631425E-2 + gy * cq14 * cq6 * sq2 * 2.3526756935E-2 -
      gx * cq2 * cq3 * sq1 * sq4 * 1.438243105603 - gx * cq2 * cq5 * sq1 * sq3 * 6.7870631425E-2 -
      gx * cq14 * sq3 * sq5 * 6.7870631425E-2 - gx * cq4 * cq6 * sq1 * sq2 * 2.3526756935E-2 +
      gx * cq1 * cq6 * sq3 * sq4 * 2.3526756935E-2 + gx * cq13 * sq5 * sq6 * 2.3526756935E-2 -
      gy * cq12 * sq3 * sq5 * 1.4653732538E-2 + gy * cq15 * sq2 * sq4 * 1.4653732538E-2 -
      gy * cq14 * sq2 * sq6 * 1.00241616695E-1 - gy * cq4 * cq5 * sq1 * sq3 * 1.4653732538E-2 +
      gy * cq3 * cq6 * sq1 * sq5 * 1.00241616695E-1 + gx * cq2 * sq1 * sq3 * sq5 * 1.4653732538E-2 -
      gx * cq5 * sq1 * sq2 * sq4 * 1.4653732538E-2 + gx * cq4 * sq1 * sq2 * sq6 * 1.00241616695E-1 -
      gx * cq1 * sq3 * sq4 * sq6 * 1.00241616695E-1 + gy * cq1 * sq2 * sq4 * sq5 * 6.7870631425E-2 -
      gy * cq4 * sq1 * sq3 * sq5 * 6.7870631425E-2 + gy * cq6 * sq1 * sq3 * sq4 * 2.3526756935E-2 +
      gy * cq3 * sq1 * sq5 * sq6 * 2.3526756935E-2 - gx * sq1 * sq2 * sq4 * sq5 * 6.7870631425E-2 -
      gy * sq1 * sq3 * sq4 * sq6 * 1.00241616695E-1 - gy * cq12 * cq3 * ( mL + 7.35522E-1 ) * ( 0.0825 ) +
      gx * cq2 * cq3 * sq1 * ( mL + 7.35522E-1 ) * ( 0.0825 ) - gx * cq14 * sq3 * ( mL + 7.35522E-1 ) * ( 0.0825 ) -
      gy * cq14 * sq2 * ( mL + 7.35522E-1 ) * ( 0.384 ) + gx * cq4 * sq1 * sq2 * ( mL + 7.35522E-1 ) * ( 0.384 ) -
      gx * cq1 * sq3 * sq4 * ( mL + 7.35522E-1 ) * ( 0.384 ) + gy * cq1 * sq2 * sq4 * ( mL + 7.35522E-1 ) * ( 0.0825 ) -
      gy * cq4 * sq1 * sq3 * ( mL + 7.35522E-1 ) * ( 0.0825 ) -
      gx * sq1 * sq2 * sq4 * ( mL + 7.35522E-1 ) * ( 0.0825 ) - gy * sq1 * sq3 * sq4 * ( mL + 7.35522E-1 ) * ( 0.384 ) -
      gx * cq2 * cq3 * cq4 * sq1 * sq5 * 6.7870631425E-2 + gx * cq2 * cq3 * cq6 * sq1 * sq4 * 2.3526756935E-2 +
      gx * cq14 * cq5 * sq3 * sq6 * 2.3526756935E-2 + gy * cq12 * cq3 * sq4 * sq6 * 1.00241616695E-1 +
      gy * cq12 * cq6 * sq3 * sq5 * 1.00241616695E-1 - gy * cq15 * cq6 * sq2 * sq4 * 1.00241616695E-1 +
      gy * cq4 * cq5 * cq6 * sq1 * sq3 * 1.00241616695E-1 + gy * cq14 * cq6 * sq2 * ( clz * mL + 4.5305948634E-2 ) -
      gx * cq2 * cq3 * sq1 * sq4 * sq6 * 1.00241616695E-1 - gx * cq2 * cq6 * sq1 * sq3 * sq5 * 1.00241616695E-1 +
      gx * cq5 * cq6 * sq1 * sq2 * sq4 * 1.00241616695E-1 + gy * cq12 * sq3 * sq5 * sq6 * 2.3526756935E-2 -
      gy * cq15 * sq2 * sq4 * sq6 * 2.3526756935E-2 + gy * cq4 * cq5 * sq1 * sq3 * sq6 * 2.3526756935E-2 -
      gx * cq4 * cq6 * sq1 * sq2 * ( clz * mL + 4.5305948634E-2 ) +
      gx * cq1 * cq6 * sq3 * sq4 * ( clz * mL + 4.5305948634E-2 ) +
      gx * cq13 * sq5 * sq6 * ( clz * mL + 4.5305948634E-2 ) - gx * cq2 * sq1 * sq3 * sq5 * sq6 * 2.3526756935E-2 +
      gx * cq5 * sq1 * sq2 * sq4 * sq6 * 2.3526756935E-2 + gy * cq6 * sq1 * sq3 * sq4 * ( clz * mL + 4.5305948634E-2 ) +
      gy * cq3 * sq1 * sq5 * sq6 * ( clz * mL + 4.5305948634E-2 ) +
      gy * cq12 * cq3 * cq4 * ( mL + 7.35522E-1 ) * ( 0.0825 ) -
      gx * cq2 * cq3 * cq4 * sq1 * ( mL + 7.35522E-1 ) * ( 0.0825 ) +
      gx * cq13 * cq6 * sq5 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
      gy * cq12 * cq3 * sq4 * ( mL + 7.35522E-1 ) * ( 0.384 ) - gx * cq13 * cq5 * cq7 * ( cly * mL - 3.127439544E-3 ) -
      gx * cq2 * cq3 * sq1 * sq4 * ( mL + 7.35522E-1 ) * ( 0.384 ) -
      gy * cq14 * sq2 * sq6 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
      gy * cq3 * cq6 * sq1 * sq5 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
      gy * cq3 * cq5 * cq7 * sq1 * ( cly * mL - 3.127439544E-3 ) +
      gx * cq4 * sq1 * sq2 * sq6 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
      gx * cq1 * sq3 * sq4 * sq6 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
      gy * sq1 * sq3 * sq4 * sq6 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
      gx * cq13 * cq5 * sq7 * ( clx * mL + 7.735484874E-3 ) + gy * cq12 * cq3 * cq4 * cq5 * 1.4653732538E-2 -
      gy * cq3 * cq5 * sq1 * sq7 * ( clx * mL + 7.735484874E-3 ) - gx * cq2 * cq3 * cq4 * cq5 * sq1 * 1.4653732538E-2 +
      gx * cq14 * cq5 * cq6 * sq3 * 1.00241616695E-1 + gy * cq12 * cq3 * cq4 * sq5 * 6.7870631425E-2 -
      gy * cq12 * cq3 * cq6 * sq4 * 2.3526756935E-2 + gx * sq1 * sq2 * sq4 * sq5 * sq7 * ( clx * mL + 7.735484874E-3 ) +
      gy * cq12 * sq3 * sq5 * sq6 * ( clz * mL + 4.5305948634E-2 ) -
      gy * cq15 * sq2 * sq4 * sq6 * ( clz * mL + 4.5305948634E-2 ) +
      gy * cq4 * cq5 * sq1 * sq3 * sq6 * ( clz * mL + 4.5305948634E-2 ) -
      gx * cq2 * sq1 * sq3 * sq5 * sq6 * ( clz * mL + 4.5305948634E-2 ) +
      gx * cq5 * sq1 * sq2 * sq4 * sq6 * ( clz * mL + 4.5305948634E-2 ) +
      gx * cq14 * cq5 * cq6 * sq3 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
      gy * cq12 * cq3 * sq4 * sq6 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
      gy * cq12 * cq6 * sq3 * sq5 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
      gy * cq15 * cq6 * sq2 * sq4 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
      gy * cq4 * cq5 * cq6 * sq1 * sq3 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
      gy * cq12 * cq5 * cq7 * sq3 * ( cly * mL - 3.127439544E-3 ) -
      gx * cq2 * cq3 * sq1 * sq4 * sq6 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
      gx * cq2 * cq6 * sq1 * sq3 * sq5 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
      gx * cq5 * cq6 * sq1 * sq2 * sq4 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
      gx * cq2 * cq5 * cq7 * sq1 * sq3 * ( cly * mL - 3.127439544E-3 ) +
      gx * cq14 * cq7 * sq3 * sq5 * ( cly * mL - 3.127439544E-3 ) -
      gx * cq13 * cq6 * sq5 * sq7 * ( cly * mL - 3.127439544E-3 ) -
      gy * cq1 * cq7 * sq2 * sq4 * sq5 * ( cly * mL - 3.127439544E-3 ) +
      gy * cq14 * sq2 * sq6 * sq7 * ( cly * mL - 3.127439544E-3 ) +
      gy * cq4 * cq7 * sq1 * sq3 * sq5 * ( cly * mL - 3.127439544E-3 ) -
      gy * cq3 * cq6 * sq1 * sq5 * sq7 * ( cly * mL - 3.127439544E-3 ) +
      gx * cq7 * sq1 * sq2 * sq4 * sq5 * ( cly * mL - 3.127439544E-3 ) -
      gx * cq4 * sq1 * sq2 * sq6 * sq7 * ( cly * mL - 3.127439544E-3 ) +
      gx * cq1 * sq3 * sq4 * sq6 * sq7 * ( cly * mL - 3.127439544E-3 ) +
      gy * sq1 * sq3 * sq4 * sq6 * sq7 * ( cly * mL - 3.127439544E-3 ) +
      gx * cq13 * cq6 * cq7 * sq5 * ( clx * mL + 7.735484874E-3 ) -
      gy * cq12 * cq3 * cq4 * cq5 * cq6 * 1.00241616695E-1 -
      gy * cq12 * cq5 * sq3 * sq7 * ( clx * mL + 7.735484874E-3 ) -
      gy * cq14 * cq7 * sq2 * sq6 * ( clx * mL + 7.735484874E-3 ) +
      gy * cq3 * cq6 * cq7 * sq1 * sq5 * ( clx * mL + 7.735484874E-3 ) +
      gx * cq2 * cq3 * cq4 * cq5 * cq6 * sq1 * 1.00241616695E-1 - gy * cq12 * cq3 * cq4 * cq5 * sq6 * 2.3526756935E-2 +
      gx * cq2 * cq5 * sq1 * sq3 * sq7 * ( clx * mL + 7.735484874E-3 ) +
      gx * cq14 * sq3 * sq5 * sq7 * ( clx * mL + 7.735484874E-3 ) +
      gx * cq4 * cq7 * sq1 * sq2 * sq6 * ( clx * mL + 7.735484874E-3 ) -
      gx * cq1 * cq7 * sq3 * sq4 * sq6 * ( clx * mL + 7.735484874E-3 ) +
      gx * cq2 * cq3 * cq4 * cq5 * sq1 * sq6 * 2.3526756935E-2 -
      gy * cq12 * cq3 * cq6 * sq4 * ( clz * mL + 4.5305948634E-2 ) -
      gy * cq1 * sq2 * sq4 * sq5 * sq7 * ( clx * mL + 7.735484874E-3 ) +
      gy * cq4 * sq1 * sq3 * sq5 * sq7 * ( clx * mL + 7.735484874E-3 ) -
      gy * cq7 * sq1 * sq3 * sq4 * sq6 * ( clx * mL + 7.735484874E-3 ) +
      gx * cq2 * cq3 * cq6 * sq1 * sq4 * ( clz * mL + 4.5305948634E-2 ) +
      gx * cq14 * cq5 * sq3 * sq6 * ( clz * mL + 4.5305948634E-2 ) -
      gy * cq12 * cq3 * cq4 * cq7 * sq5 * ( cly * mL - 3.127439544E-3 ) +
      gx * cq2 * cq3 * cq4 * cq7 * sq1 * sq5 * ( cly * mL - 3.127439544E-3 ) -
      gx * cq14 * cq5 * cq6 * sq3 * sq7 * ( cly * mL - 3.127439544E-3 ) -
      gy * cq12 * cq3 * sq4 * sq6 * sq7 * ( cly * mL - 3.127439544E-3 ) -
      gy * cq12 * cq6 * sq3 * sq5 * sq7 * ( cly * mL - 3.127439544E-3 ) +
      gy * cq15 * cq6 * sq2 * sq4 * sq7 * ( cly * mL - 3.127439544E-3 ) -
      gy * cq4 * cq5 * cq6 * sq1 * sq3 * sq7 * ( cly * mL - 3.127439544E-3 ) +
      gx * cq2 * cq3 * sq1 * sq4 * sq6 * sq7 * ( cly * mL - 3.127439544E-3 ) +
      gx * cq2 * cq6 * sq1 * sq3 * sq5 * sq7 * ( cly * mL - 3.127439544E-3 ) -
      gx * cq5 * cq6 * sq1 * sq2 * sq4 * sq7 * ( cly * mL - 3.127439544E-3 ) +
      gx * cq14 * cq5 * cq6 * cq7 * sq3 * ( clx * mL + 7.735484874E-3 ) -
      gy * cq12 * cq3 * cq4 * sq5 * sq7 * ( clx * mL + 7.735484874E-3 ) +
      gy * cq12 * cq3 * cq7 * sq4 * sq6 * ( clx * mL + 7.735484874E-3 ) +
      gy * cq12 * cq6 * cq7 * sq3 * sq5 * ( clx * mL + 7.735484874E-3 ) -
      gy * cq15 * cq6 * cq7 * sq2 * sq4 * ( clx * mL + 7.735484874E-3 ) +
      gy * cq4 * cq5 * cq6 * cq7 * sq1 * sq3 * ( clx * mL + 7.735484874E-3 ) +
      gx * cq2 * cq3 * cq4 * sq1 * sq5 * sq7 * ( clx * mL + 7.735484874E-3 ) -
      gx * cq2 * cq3 * cq7 * sq1 * sq4 * sq6 * ( clx * mL + 7.735484874E-3 ) -
      gx * cq2 * cq6 * cq7 * sq1 * sq3 * sq5 * ( clx * mL + 7.735484874E-3 ) +
      gx * cq5 * cq6 * cq7 * sq1 * sq2 * sq4 * ( clx * mL + 7.735484874E-3 ) -
      gy * cq12 * cq3 * cq4 * cq5 * sq6 * ( clz * mL + 4.5305948634E-2 ) +
      gx * cq2 * cq3 * cq4 * cq5 * sq1 * sq6 * ( clz * mL + 4.5305948634E-2 ) -
      gy * cq12 * cq3 * cq4 * cq5 * cq6 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
      gx * cq2 * cq3 * cq4 * cq5 * cq6 * sq1 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
      gy * cq12 * cq3 * cq4 * cq5 * cq6 * cq7 * ( clx * mL + 7.735484874E-3 ) +
      gx * cq2 * cq3 * cq4 * cq5 * cq6 * cq7 * sq1 * ( clx * mL + 7.735484874E-3 ) +
      gy * cq12 * cq3 * cq4 * cq5 * cq6 * sq7 * ( cly * mL - 3.127439544E-3 ) -
      gx * cq2 * cq3 * cq4 * cq5 * cq6 * sq1 * sq7 * ( cly * mL - 3.127439544E-3 );
  g[1] =
      gz * cq2 * ( -2.031994566E-3 ) + gz * sq2 * 2.871915091512 - gx * cq12 * 2.871915091512 +
      gz * cq2 * cq3 * 6.23477394872E-1 - gx * cq1 * sq2 * 2.031994566E-3 - gy * cq2 * sq1 * 2.871915091512 -
      gz * cq2 * sq3 * 2.8227094878E-2 + gz * cq4 * sq2 * 1.438243105603 - gy * sq1 * sq2 * 2.031994566E-3 -
      gz * sq2 * sq4 * 4.2939970965E-1 + gz * sq2 * ( mL + 7.35522E-1 ) * ( 0.316 ) - gx * cq12 * cq4 * 1.438243105603 -
      gz * cq2 * cq3 * cq4 * 4.2939970965E-1 + gx * cq13 * sq2 * 6.23477394872E-1 + gx * cq12 * sq4 * 4.2939970965E-1 -
      gy * cq2 * cq4 * sq1 * 1.438243105603 - gz * cq2 * cq3 * sq4 * 1.438243105603 -
      gz * cq2 * cq5 * sq3 * 6.7870631425E-2 - gz * cq4 * cq6 * sq2 * 2.3526756935E-2 -
      gx * cq1 * sq2 * sq3 * 2.8227094878E-2 + gy * cq3 * sq1 * sq2 * 6.23477394872E-1 +
      gy * cq2 * sq1 * sq4 * 4.2939970965E-1 + gz * cq2 * sq3 * sq5 * 1.4653732538E-2 -
      gz * cq5 * sq2 * sq4 * 1.4653732538E-2 + gz * cq4 * sq2 * sq6 * 1.00241616695E-1 -
      gy * sq1 * sq2 * sq3 * 2.8227094878E-2 - gz * sq2 * sq4 * sq5 * 6.7870631425E-2 -
      gx * cq12 * ( mL + 7.35522E-1 ) * ( 0.316 ) + gz * cq2 * cq3 * ( mL + 7.35522E-1 ) * ( 0.0825 ) -
      gy * cq2 * sq1 * ( mL + 7.35522E-1 ) * ( 0.316 ) + gz * cq4 * sq2 * ( mL + 7.35522E-1 ) * ( 0.384 ) -
      gz * sq2 * sq4 * ( mL + 7.35522E-1 ) * ( 0.0825 ) + gx * cq12 * cq4 * cq6 * 2.3526756935E-2 -
      gz * cq2 * cq3 * cq4 * cq5 * 1.4653732538E-2 - gx * cq13 * cq4 * sq2 * 4.2939970965E-1 +
      gx * cq12 * cq5 * sq4 * 1.4653732538E-2 - gx * cq12 * cq4 * sq6 * 1.00241616695E-1 +
      gy * cq2 * cq4 * cq6 * sq1 * 2.3526756935E-2 - gz * cq2 * cq3 * cq4 * sq5 * 6.7870631425E-2 +
      gz * cq2 * cq3 * cq6 * sq4 * 2.3526756935E-2 - gx * cq13 * sq2 * sq4 * 1.438243105603 -
      gx * cq15 * sq2 * sq3 * 6.7870631425E-2 + gx * cq12 * sq4 * sq5 * 6.7870631425E-2 -
      gy * cq3 * cq4 * sq1 * sq2 * 4.2939970965E-1 + gy * cq2 * cq5 * sq1 * sq4 * 1.4653732538E-2 -
      gy * cq2 * cq4 * sq1 * sq6 * 1.00241616695E-1 - gz * cq2 * cq3 * sq4 * sq6 * 1.00241616695E-1 -
      gz * cq2 * cq6 * sq3 * sq5 * 1.00241616695E-1 + gz * cq5 * cq6 * sq2 * sq4 * 1.00241616695E-1 -
      gz * cq4 * cq6 * sq2 * ( clz * mL + 4.5305948634E-2 ) + gx * cq1 * sq2 * sq3 * sq5 * 1.4653732538E-2 -
      gy * cq3 * sq1 * sq2 * sq4 * 1.438243105603 - gy * cq5 * sq1 * sq2 * sq3 * 6.7870631425E-2 +
      gy * cq2 * sq1 * sq4 * sq5 * 6.7870631425E-2 - gz * cq2 * sq3 * sq5 * sq6 * 2.3526756935E-2 +
      gz * cq5 * sq2 * sq4 * sq6 * 2.3526756935E-2 + gy * sq1 * sq2 * sq3 * sq5 * 1.4653732538E-2 -
      gx * cq12 * cq4 * ( mL + 7.35522E-1 ) * ( 0.384 ) - gz * cq2 * cq3 * cq4 * ( mL + 7.35522E-1 ) * ( 0.0825 ) +
      gx * cq13 * sq2 * ( mL + 7.35522E-1 ) * ( 0.0825 ) + gx * cq12 * sq4 * ( mL + 7.35522E-1 ) * ( 0.0825 ) -
      gy * cq2 * cq4 * sq1 * ( mL + 7.35522E-1 ) * ( 0.384 ) - gz * cq2 * cq3 * sq4 * ( mL + 7.35522E-1 ) * ( 0.384 ) +
      gy * cq3 * sq1 * sq2 * ( mL + 7.35522E-1 ) * ( 0.0825 ) +
      gy * cq2 * sq1 * sq4 * ( mL + 7.35522E-1 ) * ( 0.0825 ) + gz * cq4 * sq2 * sq6 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
      gx * cq12 * cq4 * cq6 * ( clz * mL + 4.5305948634E-2 ) - gx * cq13 * cq4 * sq2 * sq5 * 6.7870631425E-2 +
      gx * cq13 * cq6 * sq2 * sq4 * 2.3526756935E-2 - gx * cq12 * cq5 * sq4 * sq6 * 2.3526756935E-2 -
      gy * cq3 * cq4 * cq5 * sq1 * sq2 * 1.4653732538E-2 - gy * cq2 * cq5 * cq6 * sq1 * sq4 * 1.00241616695E-1 +
      gy * cq2 * cq4 * cq6 * sq1 * ( clz * mL + 4.5305948634E-2 ) +
      gz * cq2 * cq3 * cq6 * sq4 * ( clz * mL + 4.5305948634E-2 ) +
      gz * sq2 * sq4 * sq5 * sq7 * ( clx * mL + 7.735484874E-3 ) - gx * cq13 * sq2 * sq4 * sq6 * 1.00241616695E-1 -
      gx * cq1 * cq6 * sq2 * sq3 * sq5 * 1.00241616695E-1 - gy * cq3 * cq4 * sq1 * sq2 * sq5 * 6.7870631425E-2 +
      gy * cq3 * cq6 * sq1 * sq2 * sq4 * 2.3526756935E-2 - gy * cq2 * cq5 * sq1 * sq4 * sq6 * 2.3526756935E-2 -
      gx * cq1 * sq2 * sq3 * sq5 * sq6 * 2.3526756935E-2 - gy * cq3 * sq1 * sq2 * sq4 * sq6 * 1.00241616695E-1 -
      gy * cq6 * sq1 * sq2 * sq3 * sq5 * 1.00241616695E-1 -
      gz * cq2 * sq3 * sq5 * sq6 * ( clz * mL + 4.5305948634E-2 ) +
      gz * cq5 * sq2 * sq4 * sq6 * ( clz * mL + 4.5305948634E-2 ) - gy * sq1 * sq2 * sq3 * sq5 * sq6 * 2.3526756935E-2 -
      gx * cq13 * cq4 * sq2 * ( mL + 7.35522E-1 ) * ( 0.0825 ) -
      gx * cq12 * cq4 * sq6 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
      gx * cq13 * sq2 * sq4 * ( mL + 7.35522E-1 ) * ( 0.384 ) -
      gy * cq3 * cq4 * sq1 * sq2 * ( mL + 7.35522E-1 ) * ( 0.0825 ) -
      gy * cq2 * cq4 * sq1 * sq6 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
      gz * cq2 * cq3 * sq4 * sq6 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
      gz * cq2 * cq6 * sq3 * sq5 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
      gz * cq5 * cq6 * sq2 * sq4 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
      gz * cq2 * cq5 * cq7 * sq3 * ( cly * mL - 3.127439544E-3 ) -
      gy * cq3 * sq1 * sq2 * sq4 * ( mL + 7.35522E-1 ) * ( 0.384 ) +
      gz * cq7 * sq2 * sq4 * sq5 * ( cly * mL - 3.127439544E-3 ) -
      gz * cq4 * sq2 * sq6 * sq7 * ( cly * mL - 3.127439544E-3 ) + gz * cq2 * cq3 * cq4 * cq5 * cq6 * 1.00241616695E-1 +
      gz * cq2 * cq5 * sq3 * sq7 * ( clx * mL + 7.735484874E-3 ) +
      gz * cq4 * cq7 * sq2 * sq6 * ( clx * mL + 7.735484874E-3 ) - gx * cq13 * cq4 * cq5 * sq2 * 1.4653732538E-2 -
      gx * cq12 * cq5 * cq6 * sq4 * 1.00241616695E-1 + gz * cq2 * cq3 * cq4 * cq5 * sq6 * 2.3526756935E-2 +
      gy * cq3 * cq6 * sq1 * sq2 * sq4 * ( clz * mL + 4.5305948634E-2 ) -
      gy * cq2 * cq5 * sq1 * sq4 * sq6 * ( clz * mL + 4.5305948634E-2 ) +
      gz * cq2 * cq3 * cq4 * cq5 * cq6 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
      gx * cq1 * sq2 * sq3 * sq5 * sq6 * ( clz * mL + 4.5305948634E-2 ) -
      gx * cq12 * cq5 * cq6 * sq4 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
      gy * sq1 * sq2 * sq3 * sq5 * sq6 * ( clz * mL + 4.5305948634E-2 ) -
      gy * cq2 * cq5 * cq6 * sq1 * sq4 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
      gz * cq2 * cq3 * cq4 * cq7 * sq5 * ( cly * mL - 3.127439544E-3 ) -
      gx * cq13 * sq2 * sq4 * sq6 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
      gx * cq1 * cq6 * sq2 * sq3 * sq5 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
      gx * cq15 * cq7 * sq2 * sq3 * ( cly * mL - 3.127439544E-3 ) -
      gx * cq12 * cq7 * sq4 * sq5 * ( cly * mL - 3.127439544E-3 ) +
      gx * cq12 * cq4 * sq6 * sq7 * ( cly * mL - 3.127439544E-3 ) -
      gy * cq3 * sq1 * sq2 * sq4 * sq6 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
      gy * cq6 * sq1 * sq2 * sq3 * sq5 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
      gy * cq5 * cq7 * sq1 * sq2 * sq3 * ( cly * mL - 3.127439544E-3 ) -
      gy * cq2 * cq7 * sq1 * sq4 * sq5 * ( cly * mL - 3.127439544E-3 ) +
      gy * cq2 * cq4 * sq1 * sq6 * sq7 * ( cly * mL - 3.127439544E-3 ) +
      gz * cq2 * cq3 * sq4 * sq6 * sq7 * ( cly * mL - 3.127439544E-3 ) +
      gz * cq2 * cq6 * sq3 * sq5 * sq7 * ( cly * mL - 3.127439544E-3 ) -
      gz * cq5 * cq6 * sq2 * sq4 * sq7 * ( cly * mL - 3.127439544E-3 ) -
      gx * cq12 * cq4 * cq7 * sq6 * ( clx * mL + 7.735484874E-3 ) -
      gy * cq2 * cq4 * cq7 * sq1 * sq6 * ( clx * mL + 7.735484874E-3 ) +
      gz * cq2 * cq3 * cq4 * sq5 * sq7 * ( clx * mL + 7.735484874E-3 ) -
      gz * cq2 * cq3 * cq7 * sq4 * sq6 * ( clx * mL + 7.735484874E-3 ) -
      gz * cq2 * cq6 * cq7 * sq3 * sq5 * ( clx * mL + 7.735484874E-3 ) +
      gz * cq5 * cq6 * cq7 * sq2 * sq4 * ( clx * mL + 7.735484874E-3 ) +
      gx * cq13 * cq4 * cq5 * cq6 * sq2 * 1.00241616695E-1 +
      gx * cq15 * sq2 * sq3 * sq7 * ( clx * mL + 7.735484874E-3 ) -
      gx * cq12 * sq4 * sq5 * sq7 * ( clx * mL + 7.735484874E-3 ) +
      gx * cq13 * cq4 * cq5 * sq2 * sq6 * 2.3526756935E-2 + gy * cq3 * cq4 * cq5 * cq6 * sq1 * sq2 * 1.00241616695E-1 +
      gz * cq2 * cq3 * cq4 * cq5 * sq6 * ( clz * mL + 4.5305948634E-2 ) +
      gy * cq5 * sq1 * sq2 * sq3 * sq7 * ( clx * mL + 7.735484874E-3 ) -
      gy * cq2 * sq1 * sq4 * sq5 * sq7 * ( clx * mL + 7.735484874E-3 ) +
      gy * cq3 * cq4 * cq5 * sq1 * sq2 * sq6 * 2.3526756935E-2 +
      gx * cq13 * cq6 * sq2 * sq4 * ( clz * mL + 4.5305948634E-2 ) -
      gx * cq12 * cq5 * sq4 * sq6 * ( clz * mL + 4.5305948634E-2 ) +
      gy * cq3 * cq4 * cq5 * cq6 * sq1 * sq2 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
      gz * cq2 * cq3 * cq4 * cq5 * cq6 * sq7 * ( cly * mL - 3.127439544E-3 ) +
      gx * cq13 * cq4 * cq7 * sq2 * sq5 * ( cly * mL - 3.127439544E-3 ) +
      gx * cq12 * cq5 * cq6 * sq4 * sq7 * ( cly * mL - 3.127439544E-3 ) +
      gy * cq3 * cq4 * cq7 * sq1 * sq2 * sq5 * ( cly * mL - 3.127439544E-3 ) +
      gy * cq2 * cq5 * cq6 * sq1 * sq4 * sq7 * ( cly * mL - 3.127439544E-3 ) +
      gx * cq13 * sq2 * sq4 * sq6 * sq7 * ( cly * mL - 3.127439544E-3 ) +
      gx * cq1 * cq6 * sq2 * sq3 * sq5 * sq7 * ( cly * mL - 3.127439544E-3 ) +
      gy * cq3 * sq1 * sq2 * sq4 * sq6 * sq7 * ( cly * mL - 3.127439544E-3 ) +
      gy * cq6 * sq1 * sq2 * sq3 * sq5 * sq7 * ( cly * mL - 3.127439544E-3 ) +
      gz * cq2 * cq3 * cq4 * cq5 * cq6 * cq7 * ( clx * mL + 7.735484874E-3 ) -
      gx * cq12 * cq5 * cq6 * cq7 * sq4 * ( clx * mL + 7.735484874E-3 ) -
      gy * cq2 * cq5 * cq6 * cq7 * sq1 * sq4 * ( clx * mL + 7.735484874E-3 ) +
      gx * cq13 * cq4 * sq2 * sq5 * sq7 * ( clx * mL + 7.735484874E-3 ) -
      gx * cq13 * cq7 * sq2 * sq4 * sq6 * ( clx * mL + 7.735484874E-3 ) -
      gx * cq1 * cq6 * cq7 * sq2 * sq3 * sq5 * ( clx * mL + 7.735484874E-3 ) +
      gy * cq3 * cq4 * sq1 * sq2 * sq5 * sq7 * ( clx * mL + 7.735484874E-3 ) -
      gy * cq3 * cq7 * sq1 * sq2 * sq4 * sq6 * ( clx * mL + 7.735484874E-3 ) -
      gy * cq6 * cq7 * sq1 * sq2 * sq3 * sq5 * ( clx * mL + 7.735484874E-3 ) +
      gx * cq13 * cq4 * cq5 * sq2 * sq6 * ( clz * mL + 4.5305948634E-2 ) +
      gy * cq3 * cq4 * cq5 * sq1 * sq2 * sq6 * ( clz * mL + 4.5305948634E-2 ) +
      gx * cq13 * cq4 * cq5 * cq6 * sq2 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
      gx * cq13 * cq4 * cq5 * cq6 * cq7 * sq2 * ( clx * mL + 7.735484874E-3 ) +
      gy * cq3 * cq4 * cq5 * cq6 * cq7 * sq1 * sq2 * ( clx * mL + 7.735484874E-3 ) -
      gx * cq13 * cq4 * cq5 * cq6 * sq2 * sq7 * ( cly * mL - 3.127439544E-3 ) -
      gy * cq3 * cq4 * cq5 * cq6 * sq1 * sq2 * sq7 * ( cly * mL - 3.127439544E-3 );
  g[2] =
      gy * cq13 * ( -6.23477394872E-1 ) + gx * cq3 * sq1 * 6.23477394872E-1 + gy * cq1 * sq3 * 2.8227094878E-2 -
      gz * cq3 * sq2 * 2.8227094878E-2 - gx * sq1 * sq3 * 2.8227094878E-2 - gz * sq2 * sq3 * 6.23477394872E-1 +
      gx * cq12 * cq3 * 2.8227094878E-2 + gy * cq13 * cq4 * 4.2939970965E-1 + gx * cq12 * sq3 * 6.23477394872E-1 -
      gx * cq3 * cq4 * sq1 * 4.2939970965E-1 + gy * cq2 * cq3 * sq1 * 2.8227094878E-2 +
      gy * cq13 * sq4 * 1.438243105603 + gy * cq15 * sq3 * 6.7870631425E-2 - gz * cq3 * cq5 * sq2 * 6.7870631425E-2 -
      gx * cq3 * sq1 * sq4 * 1.438243105603 - gx * cq5 * sq1 * sq3 * 6.7870631425E-2 +
      gy * cq2 * sq1 * sq3 * 6.23477394872E-1 - gy * cq1 * sq3 * sq5 * 1.4653732538E-2 +
      gz * cq4 * sq2 * sq3 * 4.2939970965E-1 + gz * cq3 * sq2 * sq5 * 1.4653732538E-2 +
      gx * sq1 * sq3 * sq5 * 1.4653732538E-2 + gz * sq2 * sq3 * sq4 * 1.438243105603 -
      gy * cq13 * ( mL + 7.35522E-1 ) * ( 0.0825 ) + gx * cq3 * sq1 * ( mL + 7.35522E-1 ) * ( 0.0825 ) -
      gz * sq2 * sq3 * ( mL + 7.35522E-1 ) * ( 0.0825 ) + gx * cq12 * cq3 * cq5 * 6.7870631425E-2 +
      gy * cq13 * cq4 * cq5 * 1.4653732538E-2 - gx * cq12 * cq4 * sq3 * 4.2939970965E-1 -
      gx * cq12 * cq3 * sq5 * 1.4653732538E-2 - gx * cq3 * cq4 * cq5 * sq1 * 1.4653732538E-2 +
      gy * cq2 * cq3 * cq5 * sq1 * 6.7870631425E-2 + gy * cq13 * cq4 * sq5 * 6.7870631425E-2 -
      gy * cq13 * cq6 * sq4 * 2.3526756935E-2 - gx * cq12 * sq3 * sq4 * 1.438243105603 -
      gx * cq3 * cq4 * sq1 * sq5 * 6.7870631425E-2 + gx * cq3 * cq6 * sq1 * sq4 * 2.3526756935E-2 -
      gy * cq2 * cq4 * sq1 * sq3 * 4.2939970965E-1 - gy * cq2 * cq3 * sq1 * sq5 * 1.4653732538E-2 +
      gy * cq13 * sq4 * sq6 * 1.00241616695E-1 + gy * cq1 * cq6 * sq3 * sq5 * 1.00241616695E-1 +
      gz * cq4 * cq5 * sq2 * sq3 * 1.4653732538E-2 - gz * cq3 * cq6 * sq2 * sq5 * 1.00241616695E-1 -
      gx * cq3 * sq1 * sq4 * sq6 * 1.00241616695E-1 - gx * cq6 * sq1 * sq3 * sq5 * 1.00241616695E-1 -
      gy * cq2 * sq1 * sq3 * sq4 * 1.438243105603 + gy * cq1 * sq3 * sq5 * sq6 * 2.3526756935E-2 +
      gz * cq4 * sq2 * sq3 * sq5 * 6.7870631425E-2 - gz * cq6 * sq2 * sq3 * sq4 * 2.3526756935E-2 -
      gz * cq3 * sq2 * sq5 * sq6 * 2.3526756935E-2 - gx * sq1 * sq3 * sq5 * sq6 * 2.3526756935E-2 +
      gz * sq2 * sq3 * sq4 * sq6 * 1.00241616695E-1 + gy * cq13 * cq4 * ( mL + 7.35522E-1 ) * ( 0.0825 ) +
      gx * cq12 * sq3 * ( mL + 7.35522E-1 ) * ( 0.0825 ) - gx * cq3 * cq4 * sq1 * ( mL + 7.35522E-1 ) * ( 0.0825 ) +
      gy * cq13 * sq4 * ( mL + 7.35522E-1 ) * ( 0.384 ) - gx * cq3 * sq1 * sq4 * ( mL + 7.35522E-1 ) * ( 0.384 ) +
      gy * cq2 * sq1 * sq3 * ( mL + 7.35522E-1 ) * ( 0.0825 ) +
      gz * cq4 * sq2 * sq3 * ( mL + 7.35522E-1 ) * ( 0.0825 ) + gz * sq2 * sq3 * sq4 * ( mL + 7.35522E-1 ) * ( 0.384 ) +
      gx * cq5 * sq1 * sq3 * sq7 * ( clx * mL + 7.735484874E-3 ) - gx * cq12 * cq4 * sq3 * sq5 * 6.7870631425E-2 +
      gx * cq12 * cq6 * sq3 * sq4 * 2.3526756935E-2 + gx * cq12 * cq3 * sq5 * sq6 * 2.3526756935E-2 +
      gx * cq3 * cq4 * cq5 * sq1 * sq6 * 2.3526756935E-2 - gy * cq2 * cq4 * cq5 * sq1 * sq3 * 1.4653732538E-2 +
      gy * cq2 * cq3 * cq6 * sq1 * sq5 * 1.00241616695E-1 - gz * cq4 * cq5 * cq6 * sq2 * sq3 * 1.00241616695E-1 -
      gy * cq13 * cq6 * sq4 * ( clz * mL + 4.5305948634E-2 ) - gx * cq12 * sq3 * sq4 * sq6 * 1.00241616695E-1 -
      gy * cq2 * cq4 * sq1 * sq3 * sq5 * 6.7870631425E-2 + gy * cq2 * cq6 * sq1 * sq3 * sq4 * 2.3526756935E-2 +
      gy * cq2 * cq3 * sq1 * sq5 * sq6 * 2.3526756935E-2 - gz * cq4 * cq5 * sq2 * sq3 * sq6 * 2.3526756935E-2 +
      gx * cq3 * cq6 * sq1 * sq4 * ( clz * mL + 4.5305948634E-2 ) -
      gy * cq2 * sq1 * sq3 * sq4 * sq6 * 1.00241616695E-1 +
      gy * cq1 * sq3 * sq5 * sq6 * ( clz * mL + 4.5305948634E-2 ) -
      gz * cq6 * sq2 * sq3 * sq4 * ( clz * mL + 4.5305948634E-2 ) -
      gz * cq3 * sq2 * sq5 * sq6 * ( clz * mL + 4.5305948634E-2 ) -
      gx * sq1 * sq3 * sq5 * sq6 * ( clz * mL + 4.5305948634E-2 ) -
      gx * cq12 * cq4 * sq3 * ( mL + 7.35522E-1 ) * ( 0.0825 ) -
      gx * cq12 * sq3 * sq4 * ( mL + 7.35522E-1 ) * ( 0.384 ) -
      gy * cq2 * cq4 * sq1 * sq3 * ( mL + 7.35522E-1 ) * ( 0.0825 ) +
      gy * cq13 * sq4 * sq6 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
      gy * cq1 * cq6 * sq3 * sq5 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
      gz * cq3 * cq6 * sq2 * sq5 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
      gy * cq15 * cq7 * sq3 * ( cly * mL - 3.127439544E-3 ) +
      gz * cq3 * cq5 * cq7 * sq2 * ( cly * mL - 3.127439544E-3 ) -
      gx * cq3 * sq1 * sq4 * sq6 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
      gx * cq6 * sq1 * sq3 * sq5 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
      gy * cq2 * sq1 * sq3 * sq4 * ( mL + 7.35522E-1 ) * ( 0.384 ) +
      gx * cq5 * cq7 * sq1 * sq3 * ( cly * mL - 3.127439544E-3 ) +
      gz * sq2 * sq3 * sq4 * sq6 * ( mL + 7.35522E-1 ) * ( 0.088 ) - gy * cq13 * cq4 * cq5 * cq6 * 1.00241616695E-1 -
      gy * cq15 * sq3 * sq7 * ( clx * mL + 7.735484874E-3 ) +
      gz * cq3 * cq5 * sq2 * sq7 * ( clx * mL + 7.735484874E-3 ) - gx * cq12 * cq4 * cq5 * sq3 * 1.4653732538E-2 +
      gx * cq12 * cq3 * cq6 * sq5 * 1.00241616695E-1 + gx * cq3 * cq4 * cq5 * cq6 * sq1 * 1.00241616695E-1 -
      gy * cq13 * cq4 * cq5 * sq6 * 2.3526756935E-2 +
      gy * cq2 * cq6 * sq1 * sq3 * sq4 * ( clz * mL + 4.5305948634E-2 ) +
      gy * cq2 * cq3 * sq1 * sq5 * sq6 * ( clz * mL + 4.5305948634E-2 ) -
      gz * cq4 * cq5 * sq2 * sq3 * sq6 * ( clz * mL + 4.5305948634E-2 ) -
      gy * cq13 * cq4 * cq5 * cq6 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
      gx * cq12 * cq3 * cq6 * sq5 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
      gx * cq3 * cq4 * cq5 * cq6 * sq1 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
      gx * cq12 * cq3 * cq5 * cq7 * ( cly * mL - 3.127439544E-3 ) +
      gy * cq2 * cq3 * cq6 * sq1 * sq5 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
      gz * cq4 * cq5 * cq6 * sq2 * sq3 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
      gy * cq2 * cq3 * cq5 * cq7 * sq1 * ( cly * mL - 3.127439544E-3 ) -
      gy * cq13 * cq4 * cq7 * sq5 * ( cly * mL - 3.127439544E-3 ) -
      gx * cq12 * sq3 * sq4 * sq6 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
      gx * cq3 * cq4 * cq7 * sq1 * sq5 * ( cly * mL - 3.127439544E-3 ) -
      gy * cq2 * sq1 * sq3 * sq4 * sq6 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
      gy * cq13 * sq4 * sq6 * sq7 * ( cly * mL - 3.127439544E-3 ) -
      gy * cq1 * cq6 * sq3 * sq5 * sq7 * ( cly * mL - 3.127439544E-3 ) -
      gz * cq4 * cq7 * sq2 * sq3 * sq5 * ( cly * mL - 3.127439544E-3 ) +
      gz * cq3 * cq6 * sq2 * sq5 * sq7 * ( cly * mL - 3.127439544E-3 ) +
      gx * cq3 * sq1 * sq4 * sq6 * sq7 * ( cly * mL - 3.127439544E-3 ) +
      gx * cq6 * sq1 * sq3 * sq5 * sq7 * ( cly * mL - 3.127439544E-3 ) -
      gz * sq2 * sq3 * sq4 * sq6 * sq7 * ( cly * mL - 3.127439544E-3 ) -
      gx * cq12 * cq3 * cq5 * sq7 * ( clx * mL + 7.735484874E-3 ) -
      gy * cq2 * cq3 * cq5 * sq1 * sq7 * ( clx * mL + 7.735484874E-3 ) -
      gy * cq13 * cq4 * sq5 * sq7 * ( clx * mL + 7.735484874E-3 ) +
      gy * cq13 * cq7 * sq4 * sq6 * ( clx * mL + 7.735484874E-3 ) +
      gy * cq1 * cq6 * cq7 * sq3 * sq5 * ( clx * mL + 7.735484874E-3 ) -
      gz * cq3 * cq6 * cq7 * sq2 * sq5 * ( clx * mL + 7.735484874E-3 ) +
      gx * cq12 * cq4 * cq5 * cq6 * sq3 * 1.00241616695E-1 +
      gx * cq3 * cq4 * sq1 * sq5 * sq7 * ( clx * mL + 7.735484874E-3 ) -
      gx * cq3 * cq7 * sq1 * sq4 * sq6 * ( clx * mL + 7.735484874E-3 ) -
      gx * cq6 * cq7 * sq1 * sq3 * sq5 * ( clx * mL + 7.735484874E-3 ) +
      gx * cq12 * cq4 * cq5 * sq3 * sq6 * 2.3526756935E-2 + gy * cq2 * cq4 * cq5 * cq6 * sq1 * sq3 * 1.00241616695E-1 -
      gy * cq13 * cq4 * cq5 * sq6 * ( clz * mL + 4.5305948634E-2 ) -
      gz * cq4 * sq2 * sq3 * sq5 * sq7 * ( clx * mL + 7.735484874E-3 ) +
      gz * cq7 * sq2 * sq3 * sq4 * sq6 * ( clx * mL + 7.735484874E-3 ) +
      gy * cq2 * cq4 * cq5 * sq1 * sq3 * sq6 * 2.3526756935E-2 +
      gx * cq12 * cq6 * sq3 * sq4 * ( clz * mL + 4.5305948634E-2 ) +
      gx * cq12 * cq3 * sq5 * sq6 * ( clz * mL + 4.5305948634E-2 ) +
      gx * cq3 * cq4 * cq5 * sq1 * sq6 * ( clz * mL + 4.5305948634E-2 ) +
      gy * cq2 * cq4 * cq5 * cq6 * sq1 * sq3 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
      gy * cq13 * cq4 * cq5 * cq6 * sq7 * ( cly * mL - 3.127439544E-3 ) +
      gx * cq12 * cq4 * cq7 * sq3 * sq5 * ( cly * mL - 3.127439544E-3 ) -
      gx * cq12 * cq3 * cq6 * sq5 * sq7 * ( cly * mL - 3.127439544E-3 ) -
      gx * cq3 * cq4 * cq5 * cq6 * sq1 * sq7 * ( cly * mL - 3.127439544E-3 ) +
      gy * cq2 * cq4 * cq7 * sq1 * sq3 * sq5 * ( cly * mL - 3.127439544E-3 ) -
      gy * cq2 * cq3 * cq6 * sq1 * sq5 * sq7 * ( cly * mL - 3.127439544E-3 ) +
      gz * cq4 * cq5 * cq6 * sq2 * sq3 * sq7 * ( cly * mL - 3.127439544E-3 ) +
      gx * cq12 * sq3 * sq4 * sq6 * sq7 * ( cly * mL - 3.127439544E-3 ) +
      gy * cq2 * sq1 * sq3 * sq4 * sq6 * sq7 * ( cly * mL - 3.127439544E-3 ) -
      gy * cq13 * cq4 * cq5 * cq6 * cq7 * ( clx * mL + 7.735484874E-3 ) +
      gx * cq12 * cq3 * cq6 * cq7 * sq5 * ( clx * mL + 7.735484874E-3 ) +
      gx * cq3 * cq4 * cq5 * cq6 * cq7 * sq1 * ( clx * mL + 7.735484874E-3 ) +
      gy * cq2 * cq3 * cq6 * cq7 * sq1 * sq5 * ( clx * mL + 7.735484874E-3 ) -
      gz * cq4 * cq5 * cq6 * cq7 * sq2 * sq3 * ( clx * mL + 7.735484874E-3 ) +
      gx * cq12 * cq4 * sq3 * sq5 * sq7 * ( clx * mL + 7.735484874E-3 ) -
      gx * cq12 * cq7 * sq3 * sq4 * sq6 * ( clx * mL + 7.735484874E-3 ) +
      gy * cq2 * cq4 * sq1 * sq3 * sq5 * sq7 * ( clx * mL + 7.735484874E-3 ) -
      gy * cq2 * cq7 * sq1 * sq3 * sq4 * sq6 * ( clx * mL + 7.735484874E-3 ) +
      gx * cq12 * cq4 * cq5 * sq3 * sq6 * ( clz * mL + 4.5305948634E-2 ) +
      gy * cq2 * cq4 * cq5 * sq1 * sq3 * sq6 * ( clz * mL + 4.5305948634E-2 ) +
      gx * cq12 * cq4 * cq5 * cq6 * sq3 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
      gx * cq12 * cq4 * cq5 * cq6 * cq7 * sq3 * ( clx * mL + 7.735484874E-3 ) +
      gy * cq2 * cq4 * cq5 * cq6 * cq7 * sq1 * sq3 * ( clx * mL + 7.735484874E-3 ) -
      gx * cq12 * cq4 * cq5 * cq6 * sq3 * sq7 * ( cly * mL - 3.127439544E-3 ) -
      gy * cq2 * cq4 * cq5 * cq6 * sq1 * sq3 * sq7 * ( cly * mL - 3.127439544E-3 );
  g[3] =
      gz * cq2 * cq4 * 4.2939970965E-1 + gz * cq2 * sq4 * 1.438243105603 + gz * cq2 * cq4 * cq5 * 1.4653732538E-2 +
      gx * cq14 * sq2 * 4.2939970965E-1 + gy * cq14 * sq3 * 1.438243105603 - gz * cq3 * cq4 * sq2 * 1.438243105603 +
      gz * cq2 * cq4 * sq5 * 6.7870631425E-2 - gz * cq2 * cq6 * sq4 * 2.3526756935E-2 +
      gx * cq1 * sq2 * sq4 * 1.438243105603 - gx * cq4 * sq1 * sq3 * 1.438243105603 +
      gy * cq4 * sq1 * sq2 * 4.2939970965E-1 - gy * cq1 * sq3 * sq4 * 4.2939970965E-1 +
      gz * cq3 * sq2 * sq4 * 4.2939970965E-1 + gz * cq2 * sq4 * sq6 * 1.00241616695E-1 +
      gx * sq1 * sq3 * sq4 * 4.2939970965E-1 + gy * sq1 * sq2 * sq4 * 1.438243105603 +
      gz * cq2 * cq4 * ( mL + 7.35522E-1 ) * ( 0.0825 ) + gz * cq2 * sq4 * ( mL + 7.35522E-1 ) * ( 0.384 ) +
      gx * cq12 * cq3 * cq4 * 1.438243105603 - gz * cq2 * cq4 * cq5 * cq6 * 1.00241616695E-1 -
      gx * cq12 * cq3 * sq4 * 4.2939970965E-1 + gx * cq14 * cq5 * sq2 * 1.4653732538E-2 +
      gy * cq2 * cq3 * cq4 * sq1 * 1.438243105603 - gy * cq14 * cq6 * sq3 * 2.3526756935E-2 +
      gz * cq3 * cq4 * cq6 * sq2 * 2.3526756935E-2 - gz * cq2 * cq4 * cq5 * sq6 * 2.3526756935E-2 +
      gx * cq14 * sq2 * sq5 * 6.7870631425E-2 - gx * cq1 * cq6 * sq2 * sq4 * 2.3526756935E-2 +
      gx * cq4 * cq6 * sq1 * sq3 * 2.3526756935E-2 - gy * cq2 * cq3 * sq1 * sq4 * 4.2939970965E-1 +
      gy * cq4 * cq5 * sq1 * sq2 * 1.4653732538E-2 - gy * cq15 * sq3 * sq4 * 1.4653732538E-2 +
      gy * cq14 * sq3 * sq6 * 1.00241616695E-1 + gz * cq3 * cq5 * sq2 * sq4 * 1.4653732538E-2 -
      gz * cq3 * cq4 * sq2 * sq6 * 1.00241616695E-1 - gz * cq2 * cq6 * sq4 * ( clz * mL + 4.5305948634E-2 ) +
      gx * cq1 * sq2 * sq4 * sq6 * 1.00241616695E-1 + gx * cq5 * sq1 * sq3 * sq4 * 1.4653732538E-2 -
      gx * cq4 * sq1 * sq3 * sq6 * 1.00241616695E-1 + gy * cq4 * sq1 * sq2 * sq5 * 6.7870631425E-2 -
      gy * cq1 * sq3 * sq4 * sq5 * 6.7870631425E-2 - gy * cq6 * sq1 * sq2 * sq4 * 2.3526756935E-2 +
      gz * cq3 * sq2 * sq4 * sq5 * 6.7870631425E-2 + gx * sq1 * sq3 * sq4 * sq5 * 6.7870631425E-2 +
      gy * sq1 * sq2 * sq4 * sq6 * 1.00241616695E-1 + gx * cq14 * sq2 * ( mL + 7.35522E-1 ) * ( 0.0825 ) +
      gy * cq14 * sq3 * ( mL + 7.35522E-1 ) * ( 0.384 ) - gz * cq3 * cq4 * sq2 * ( mL + 7.35522E-1 ) * ( 0.384 ) +
      gx * cq1 * sq2 * sq4 * ( mL + 7.35522E-1 ) * ( 0.384 ) - gx * cq4 * sq1 * sq3 * ( mL + 7.35522E-1 ) * ( 0.384 ) +
      gy * cq4 * sq1 * sq2 * ( mL + 7.35522E-1 ) * ( 0.0825 ) -
      gy * cq1 * sq3 * sq4 * ( mL + 7.35522E-1 ) * ( 0.0825 ) +
      gz * cq3 * sq2 * sq4 * ( mL + 7.35522E-1 ) * ( 0.0825 ) + gz * cq2 * sq4 * sq6 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
      gx * sq1 * sq3 * sq4 * ( mL + 7.35522E-1 ) * ( 0.0825 ) + gy * sq1 * sq2 * sq4 * ( mL + 7.35522E-1 ) * ( 0.384 ) -
      gx * cq12 * cq3 * sq4 * sq5 * 6.7870631425E-2 - gx * cq14 * cq5 * sq2 * sq6 * 2.3526756935E-2 -
      gy * cq2 * cq3 * cq5 * sq1 * sq4 * 1.4653732538E-2 + gy * cq2 * cq3 * cq4 * sq1 * sq6 * 1.00241616695E-1 -
      gy * cq4 * cq5 * cq6 * sq1 * sq2 * 1.00241616695E-1 + gy * cq15 * cq6 * sq3 * sq4 * 1.00241616695E-1 -
      gz * cq3 * cq5 * cq6 * sq2 * sq4 * 1.00241616695E-1 - gy * cq14 * cq6 * sq3 * ( clz * mL + 4.5305948634E-2 ) +
      gz * cq3 * cq4 * cq6 * sq2 * ( clz * mL + 4.5305948634E-2 ) -
      gz * cq2 * cq4 * cq5 * sq6 * ( clz * mL + 4.5305948634E-2 ) -
      gx * cq5 * cq6 * sq1 * sq3 * sq4 * 1.00241616695E-1 - gy * cq2 * cq3 * sq1 * sq4 * sq5 * 6.7870631425E-2 -
      gy * cq4 * cq5 * sq1 * sq2 * sq6 * 2.3526756935E-2 + gy * cq15 * sq3 * sq4 * sq6 * 2.3526756935E-2 -
      gz * cq3 * cq5 * sq2 * sq4 * sq6 * 2.3526756935E-2 - gx * cq1 * cq6 * sq2 * sq4 * ( clz * mL + 4.5305948634E-2 ) +
      gx * cq4 * cq6 * sq1 * sq3 * ( clz * mL + 4.5305948634E-2 ) - gx * cq5 * sq1 * sq3 * sq4 * sq6 * 2.3526756935E-2 -
      gy * cq6 * sq1 * sq2 * sq4 * ( clz * mL + 4.5305948634E-2 ) +
      gx * cq12 * cq3 * cq4 * ( mL + 7.35522E-1 ) * ( 0.384 ) -
      gz * cq2 * cq4 * cq5 * cq6 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
      gx * cq12 * cq3 * sq4 * ( mL + 7.35522E-1 ) * ( 0.0825 ) +
      gy * cq2 * cq3 * cq4 * sq1 * ( mL + 7.35522E-1 ) * ( 0.384 ) -
      gy * cq2 * cq3 * sq1 * sq4 * ( mL + 7.35522E-1 ) * ( 0.0825 ) +
      gy * cq14 * sq3 * sq6 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
      gz * cq3 * cq4 * sq2 * sq6 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
      gz * cq2 * cq4 * cq7 * sq5 * ( cly * mL - 3.127439544E-3 ) +
      gx * cq1 * sq2 * sq4 * sq6 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
      gx * cq4 * sq1 * sq3 * sq6 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
      gy * sq1 * sq2 * sq4 * sq6 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
      gz * cq2 * sq4 * sq6 * sq7 * ( cly * mL - 3.127439544E-3 ) - gx * cq12 * cq3 * cq4 * cq6 * 2.3526756935E-2 -
      gz * cq2 * cq4 * sq5 * sq7 * ( clx * mL + 7.735484874E-3 ) +
      gz * cq2 * cq7 * sq4 * sq6 * ( clx * mL + 7.735484874E-3 ) - gx * cq12 * cq3 * cq5 * sq4 * 1.4653732538E-2 +
      gx * cq12 * cq3 * cq4 * sq6 * 1.00241616695E-1 - gx * cq14 * cq5 * cq6 * sq2 * 1.00241616695E-1 -
      gy * cq2 * cq3 * cq4 * cq6 * sq1 * 2.3526756935E-2 -
      gx * sq1 * sq3 * sq4 * sq5 * sq7 * ( clx * mL + 7.735484874E-3 ) -
      gy * cq4 * cq5 * sq1 * sq2 * sq6 * ( clz * mL + 4.5305948634E-2 ) +
      gy * cq15 * sq3 * sq4 * sq6 * ( clz * mL + 4.5305948634E-2 ) -
      gz * cq3 * cq5 * sq2 * sq4 * sq6 * ( clz * mL + 4.5305948634E-2 ) -
      gx * cq5 * sq1 * sq3 * sq4 * sq6 * ( clz * mL + 4.5305948634E-2 ) +
      gx * cq12 * cq3 * cq4 * sq6 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
      gx * cq14 * cq5 * cq6 * sq2 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
      gy * cq2 * cq3 * cq4 * sq1 * sq6 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
      gy * cq4 * cq5 * cq6 * sq1 * sq2 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
      gy * cq15 * cq6 * sq3 * sq4 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
      gz * cq3 * cq5 * cq6 * sq2 * sq4 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
      gz * cq2 * cq4 * cq5 * cq6 * sq7 * ( cly * mL - 3.127439544E-3 ) -
      gx * cq5 * cq6 * sq1 * sq3 * sq4 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
      gx * cq14 * cq7 * sq2 * sq5 * ( cly * mL - 3.127439544E-3 ) -
      gy * cq4 * cq7 * sq1 * sq2 * sq5 * ( cly * mL - 3.127439544E-3 ) +
      gy * cq1 * cq7 * sq3 * sq4 * sq5 * ( cly * mL - 3.127439544E-3 ) -
      gy * cq14 * sq3 * sq6 * sq7 * ( cly * mL - 3.127439544E-3 ) -
      gz * cq3 * cq7 * sq2 * sq4 * sq5 * ( cly * mL - 3.127439544E-3 ) +
      gz * cq3 * cq4 * sq2 * sq6 * sq7 * ( cly * mL - 3.127439544E-3 ) -
      gx * cq1 * sq2 * sq4 * sq6 * sq7 * ( cly * mL - 3.127439544E-3 ) -
      gx * cq7 * sq1 * sq3 * sq4 * sq5 * ( cly * mL - 3.127439544E-3 ) +
      gx * cq4 * sq1 * sq3 * sq6 * sq7 * ( cly * mL - 3.127439544E-3 ) -
      gy * sq1 * sq2 * sq4 * sq6 * sq7 * ( cly * mL - 3.127439544E-3 ) -
      gz * cq2 * cq4 * cq5 * cq6 * cq7 * ( clx * mL + 7.735484874E-3 ) +
      gy * cq14 * cq7 * sq3 * sq6 * ( clx * mL + 7.735484874E-3 ) -
      gz * cq3 * cq4 * cq7 * sq2 * sq6 * ( clx * mL + 7.735484874E-3 ) +
      gx * cq12 * cq3 * cq5 * cq6 * sq4 * 1.00241616695E-1 -
      gx * cq12 * cq3 * cq4 * cq6 * ( clz * mL + 4.5305948634E-2 ) -
      gx * cq14 * sq2 * sq5 * sq7 * ( clx * mL + 7.735484874E-3 ) +
      gx * cq1 * cq7 * sq2 * sq4 * sq6 * ( clx * mL + 7.735484874E-3 ) -
      gx * cq4 * cq7 * sq1 * sq3 * sq6 * ( clx * mL + 7.735484874E-3 ) +
      gx * cq12 * cq3 * cq5 * sq4 * sq6 * 2.3526756935E-2 + gy * cq2 * cq3 * cq5 * cq6 * sq1 * sq4 * 1.00241616695E-1 -
      gy * cq2 * cq3 * cq4 * cq6 * sq1 * ( clz * mL + 4.5305948634E-2 ) -
      gy * cq4 * sq1 * sq2 * sq5 * sq7 * ( clx * mL + 7.735484874E-3 ) +
      gy * cq1 * sq3 * sq4 * sq5 * sq7 * ( clx * mL + 7.735484874E-3 ) +
      gy * cq7 * sq1 * sq2 * sq4 * sq6 * ( clx * mL + 7.735484874E-3 ) -
      gz * cq3 * sq2 * sq4 * sq5 * sq7 * ( clx * mL + 7.735484874E-3 ) +
      gy * cq2 * cq3 * cq5 * sq1 * sq4 * sq6 * 2.3526756935E-2 -
      gx * cq14 * cq5 * sq2 * sq6 * ( clz * mL + 4.5305948634E-2 ) +
      gy * cq2 * cq3 * cq5 * cq6 * sq1 * sq4 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
      gx * cq12 * cq3 * cq7 * sq4 * sq5 * ( cly * mL - 3.127439544E-3 ) -
      gx * cq12 * cq3 * cq4 * sq6 * sq7 * ( cly * mL - 3.127439544E-3 ) +
      gx * cq14 * cq5 * cq6 * sq2 * sq7 * ( cly * mL - 3.127439544E-3 ) +
      gy * cq2 * cq3 * cq7 * sq1 * sq4 * sq5 * ( cly * mL - 3.127439544E-3 ) -
      gy * cq2 * cq3 * cq4 * sq1 * sq6 * sq7 * ( cly * mL - 3.127439544E-3 ) +
      gy * cq4 * cq5 * cq6 * sq1 * sq2 * sq7 * ( cly * mL - 3.127439544E-3 ) -
      gy * cq15 * cq6 * sq3 * sq4 * sq7 * ( cly * mL - 3.127439544E-3 ) +
      gz * cq3 * cq5 * cq6 * sq2 * sq4 * sq7 * ( cly * mL - 3.127439544E-3 ) +
      gx * cq5 * cq6 * sq1 * sq3 * sq4 * sq7 * ( cly * mL - 3.127439544E-3 ) +
      gx * cq12 * cq3 * cq4 * cq7 * sq6 * ( clx * mL + 7.735484874E-3 ) -
      gx * cq14 * cq5 * cq6 * cq7 * sq2 * ( clx * mL + 7.735484874E-3 ) +
      gy * cq2 * cq3 * cq4 * cq7 * sq1 * sq6 * ( clx * mL + 7.735484874E-3 ) -
      gy * cq4 * cq5 * cq6 * cq7 * sq1 * sq2 * ( clx * mL + 7.735484874E-3 ) +
      gy * cq15 * cq6 * cq7 * sq3 * sq4 * ( clx * mL + 7.735484874E-3 ) -
      gz * cq3 * cq5 * cq6 * cq7 * sq2 * sq4 * ( clx * mL + 7.735484874E-3 ) +
      gx * cq12 * cq3 * sq4 * sq5 * sq7 * ( clx * mL + 7.735484874E-3 ) -
      gx * cq5 * cq6 * cq7 * sq1 * sq3 * sq4 * ( clx * mL + 7.735484874E-3 ) +
      gy * cq2 * cq3 * sq1 * sq4 * sq5 * sq7 * ( clx * mL + 7.735484874E-3 ) +
      gx * cq12 * cq3 * cq5 * sq4 * sq6 * ( clz * mL + 4.5305948634E-2 ) +
      gy * cq2 * cq3 * cq5 * sq1 * sq4 * sq6 * ( clz * mL + 4.5305948634E-2 ) +
      gx * cq12 * cq3 * cq5 * cq6 * sq4 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
      gx * cq12 * cq3 * cq5 * cq6 * cq7 * sq4 * ( clx * mL + 7.735484874E-3 ) +
      gy * cq2 * cq3 * cq5 * cq6 * cq7 * sq1 * sq4 * ( clx * mL + 7.735484874E-3 ) -
      gx * cq12 * cq3 * cq5 * cq6 * sq4 * sq7 * ( cly * mL - 3.127439544E-3 ) -
      gy * cq2 * cq3 * cq5 * cq6 * sq1 * sq4 * sq7 * ( cly * mL - 3.127439544E-3 );
  g[4] =
      gy * cq13 * cq5 * 1.4653732538E-2 - gx * cq3 * cq5 * sq1 * 1.4653732538E-2 + gy * cq13 * sq5 * 6.7870631425E-2 +
      gz * cq2 * cq5 * sq4 * 6.7870631425E-2 - gx * cq3 * sq1 * sq5 * 6.7870631425E-2 +
      gz * cq5 * sq2 * sq3 * 1.4653732538E-2 - gz * cq2 * sq4 * sq5 * 1.4653732538E-2 +
      gz * sq2 * sq3 * sq5 * 6.7870631425E-2 - gy * cq13 * cq5 * cq6 * 1.00241616695E-1 -
      gx * cq12 * cq5 * sq3 * 1.4653732538E-2 + gx * cq3 * cq5 * cq6 * sq1 * 1.00241616695E-1 +
      gy * cq14 * cq5 * sq3 * 6.7870631425E-2 - gy * cq13 * cq5 * sq6 * 2.3526756935E-2 -
      gz * cq3 * cq4 * cq5 * sq2 * 6.7870631425E-2 - gx * cq12 * sq3 * sq5 * 6.7870631425E-2 +
      gx * cq15 * sq2 * sq4 * 6.7870631425E-2 - gx * cq4 * cq5 * sq1 * sq3 * 6.7870631425E-2 +
      gx * cq3 * cq5 * sq1 * sq6 * 2.3526756935E-2 - gy * cq2 * cq5 * sq1 * sq3 * 1.4653732538E-2 -
      gy * cq14 * sq3 * sq5 * 1.4653732538E-2 + gz * cq3 * cq4 * sq2 * sq5 * 1.4653732538E-2 -
      gz * cq5 * cq6 * sq2 * sq3 * 1.00241616695E-1 + gz * cq2 * cq6 * sq4 * sq5 * 1.00241616695E-1 -
      gx * cq1 * sq2 * sq4 * sq5 * 1.4653732538E-2 + gx * cq4 * sq1 * sq3 * sq5 * 1.4653732538E-2 -
      gy * cq2 * sq1 * sq3 * sq5 * 6.7870631425E-2 + gy * cq5 * sq1 * sq2 * sq4 * 6.7870631425E-2 -
      gz * cq5 * sq2 * sq3 * sq6 * 2.3526756935E-2 + gz * cq2 * sq4 * sq5 * sq6 * 2.3526756935E-2 -
      gy * sq1 * sq2 * sq4 * sq5 * 1.4653732538E-2 + gx * cq3 * sq1 * sq5 * sq7 * ( clx * mL + 7.735484874E-3 ) +
      gx * cq12 * cq5 * sq3 * sq6 * 2.3526756935E-2 - gy * cq2 * cq3 * cq4 * sq1 * sq5 * 1.4653732538E-2 +
      gy * cq2 * cq5 * cq6 * sq1 * sq3 * 1.00241616695E-1 + gy * cq14 * cq6 * sq3 * sq5 * 1.00241616695E-1 -
      gz * cq3 * cq4 * cq6 * sq2 * sq5 * 1.00241616695E-1 - gy * cq13 * cq5 * sq6 * ( clz * mL + 4.5305948634E-2 ) -
      gz * sq2 * sq3 * sq5 * sq7 * ( clx * mL + 7.735484874E-3 ) + gx * cq1 * cq6 * sq2 * sq4 * sq5 * 1.00241616695E-1 -
      gx * cq4 * cq6 * sq1 * sq3 * sq5 * 1.00241616695E-1 + gy * cq2 * cq5 * sq1 * sq3 * sq6 * 2.3526756935E-2 +
      gy * cq14 * sq3 * sq5 * sq6 * 2.3526756935E-2 - gz * cq3 * cq4 * sq2 * sq5 * sq6 * 2.3526756935E-2 +
      gx * cq3 * cq5 * sq1 * sq6 * ( clz * mL + 4.5305948634E-2 ) + gx * cq1 * sq2 * sq4 * sq5 * sq6 * 2.3526756935E-2 -
      gx * cq4 * sq1 * sq3 * sq5 * sq6 * 2.3526756935E-2 + gy * cq6 * sq1 * sq2 * sq4 * sq5 * 1.00241616695E-1 -
      gz * cq5 * sq2 * sq3 * sq6 * ( clz * mL + 4.5305948634E-2 ) +
      gz * cq2 * sq4 * sq5 * sq6 * ( clz * mL + 4.5305948634E-2 ) + gy * sq1 * sq2 * sq4 * sq5 * sq6 * 2.3526756935E-2 -
      gy * cq13 * cq5 * cq6 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
      gx * cq3 * cq5 * cq6 * sq1 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
      gz * cq5 * cq6 * sq2 * sq3 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
      gz * cq2 * cq6 * sq4 * sq5 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
      gy * cq13 * cq7 * sq5 * ( cly * mL - 3.127439544E-3 ) -
      gz * cq2 * cq5 * cq7 * sq4 * ( cly * mL - 3.127439544E-3 ) +
      gx * cq3 * cq7 * sq1 * sq5 * ( cly * mL - 3.127439544E-3 ) -
      gz * cq7 * sq2 * sq3 * sq5 * ( cly * mL - 3.127439544E-3 ) + gx * cq12 * cq3 * cq4 * cq5 * 6.7870631425E-2 -
      gy * cq13 * sq5 * sq7 * ( clx * mL + 7.735484874E-3 ) -
      gz * cq2 * cq5 * sq4 * sq7 * ( clx * mL + 7.735484874E-3 ) - gx * cq12 * cq3 * cq4 * sq5 * 1.4653732538E-2 +
      gx * cq12 * cq5 * cq6 * sq3 * 1.00241616695E-1 + gy * cq2 * cq3 * cq4 * cq5 * sq1 * 6.7870631425E-2 +
      gy * cq2 * cq5 * sq1 * sq3 * sq6 * ( clz * mL + 4.5305948634E-2 ) +
      gy * cq14 * sq3 * sq5 * sq6 * ( clz * mL + 4.5305948634E-2 ) -
      gz * cq3 * cq4 * sq2 * sq5 * sq6 * ( clz * mL + 4.5305948634E-2 ) +
      gx * cq1 * sq2 * sq4 * sq5 * sq6 * ( clz * mL + 4.5305948634E-2 ) -
      gx * cq4 * sq1 * sq3 * sq5 * sq6 * ( clz * mL + 4.5305948634E-2 ) +
      gx * cq12 * cq5 * cq6 * sq3 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
      gy * sq1 * sq2 * sq4 * sq5 * sq6 * ( clz * mL + 4.5305948634E-2 ) +
      gy * cq2 * cq5 * cq6 * sq1 * sq3 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
      gy * cq14 * cq6 * sq3 * sq5 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
      gz * cq3 * cq4 * cq6 * sq2 * sq5 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
      gy * cq14 * cq5 * cq7 * sq3 * ( cly * mL - 3.127439544E-3 ) +
      gy * cq13 * cq5 * cq6 * sq7 * ( cly * mL - 3.127439544E-3 ) +
      gz * cq3 * cq4 * cq5 * cq7 * sq2 * ( cly * mL - 3.127439544E-3 ) +
      gx * cq1 * cq6 * sq2 * sq4 * sq5 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
      gx * cq4 * cq6 * sq1 * sq3 * sq5 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
      gx * cq12 * cq7 * sq3 * sq5 * ( cly * mL - 3.127439544E-3 ) -
      gx * cq15 * cq7 * sq2 * sq4 * ( cly * mL - 3.127439544E-3 ) +
      gx * cq4 * cq5 * cq7 * sq1 * sq3 * ( cly * mL - 3.127439544E-3 ) -
      gx * cq3 * cq5 * cq6 * sq1 * sq7 * ( cly * mL - 3.127439544E-3 ) +
      gy * cq6 * sq1 * sq2 * sq4 * sq5 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
      gy * cq2 * cq7 * sq1 * sq3 * sq5 * ( cly * mL - 3.127439544E-3 ) -
      gy * cq5 * cq7 * sq1 * sq2 * sq4 * ( cly * mL - 3.127439544E-3 ) +
      gz * cq5 * cq6 * sq2 * sq3 * sq7 * ( cly * mL - 3.127439544E-3 ) -
      gz * cq2 * cq6 * sq4 * sq5 * sq7 * ( cly * mL - 3.127439544E-3 ) -
      gy * cq13 * cq5 * cq6 * cq7 * ( clx * mL + 7.735484874E-3 ) +
      gx * cq3 * cq5 * cq6 * cq7 * sq1 * ( clx * mL + 7.735484874E-3 ) -
      gy * cq14 * cq5 * sq3 * sq7 * ( clx * mL + 7.735484874E-3 ) +
      gz * cq3 * cq4 * cq5 * sq2 * sq7 * ( clx * mL + 7.735484874E-3 ) -
      gz * cq5 * cq6 * cq7 * sq2 * sq3 * ( clx * mL + 7.735484874E-3 ) +
      gz * cq2 * cq6 * cq7 * sq4 * sq5 * ( clx * mL + 7.735484874E-3 ) +
      gx * cq12 * cq3 * cq4 * cq6 * sq5 * 1.00241616695E-1 +
      gx * cq12 * sq3 * sq5 * sq7 * ( clx * mL + 7.735484874E-3 ) -
      gx * cq15 * sq2 * sq4 * sq7 * ( clx * mL + 7.735484874E-3 ) +
      gx * cq4 * cq5 * sq1 * sq3 * sq7 * ( clx * mL + 7.735484874E-3 ) +
      gx * cq12 * cq3 * cq4 * sq5 * sq6 * 2.3526756935E-2 + gy * cq2 * cq3 * cq4 * cq6 * sq1 * sq5 * 1.00241616695E-1 +
      gy * cq2 * sq1 * sq3 * sq5 * sq7 * ( clx * mL + 7.735484874E-3 ) -
      gy * cq5 * sq1 * sq2 * sq4 * sq7 * ( clx * mL + 7.735484874E-3 ) +
      gy * cq2 * cq3 * cq4 * sq1 * sq5 * sq6 * 2.3526756935E-2 +
      gx * cq12 * cq5 * sq3 * sq6 * ( clz * mL + 4.5305948634E-2 ) +
      gy * cq2 * cq3 * cq4 * cq6 * sq1 * sq5 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
      gy * cq2 * cq3 * cq4 * cq5 * cq7 * sq1 * ( cly * mL - 3.127439544E-3 ) -
      gx * cq12 * cq5 * cq6 * sq3 * sq7 * ( cly * mL - 3.127439544E-3 ) -
      gy * cq2 * cq5 * cq6 * sq1 * sq3 * sq7 * ( cly * mL - 3.127439544E-3 ) -
      gy * cq14 * cq6 * sq3 * sq5 * sq7 * ( cly * mL - 3.127439544E-3 ) +
      gz * cq3 * cq4 * cq6 * sq2 * sq5 * sq7 * ( cly * mL - 3.127439544E-3 ) -
      gx * cq1 * cq6 * sq2 * sq4 * sq5 * sq7 * ( cly * mL - 3.127439544E-3 ) +
      gx * cq4 * cq6 * sq1 * sq3 * sq5 * sq7 * ( cly * mL - 3.127439544E-3 ) -
      gy * cq6 * sq1 * sq2 * sq4 * sq5 * sq7 * ( cly * mL - 3.127439544E-3 ) -
      gx * cq12 * cq3 * cq4 * cq5 * sq7 * ( clx * mL + 7.735484874E-3 ) +
      gx * cq12 * cq5 * cq6 * cq7 * sq3 * ( clx * mL + 7.735484874E-3 ) -
      gy * cq2 * cq3 * cq4 * cq5 * sq1 * sq7 * ( clx * mL + 7.735484874E-3 ) +
      gy * cq2 * cq5 * cq6 * cq7 * sq1 * sq3 * ( clx * mL + 7.735484874E-3 ) +
      gy * cq14 * cq6 * cq7 * sq3 * sq5 * ( clx * mL + 7.735484874E-3 ) -
      gz * cq3 * cq4 * cq6 * cq7 * sq2 * sq5 * ( clx * mL + 7.735484874E-3 ) +
      gx * cq1 * cq6 * cq7 * sq2 * sq4 * sq5 * ( clx * mL + 7.735484874E-3 ) -
      gx * cq4 * cq6 * cq7 * sq1 * sq3 * sq5 * ( clx * mL + 7.735484874E-3 ) +
      gy * cq6 * cq7 * sq1 * sq2 * sq4 * sq5 * ( clx * mL + 7.735484874E-3 ) +
      gx * cq12 * cq3 * cq4 * sq5 * sq6 * ( clz * mL + 4.5305948634E-2 ) +
      gy * cq2 * cq3 * cq4 * sq1 * sq5 * sq6 * ( clz * mL + 4.5305948634E-2 ) +
      gx * cq12 * cq3 * cq4 * cq6 * sq5 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
      gx * cq12 * cq3 * cq4 * cq5 * cq7 * ( cly * mL - 3.127439544E-3 ) +
      gx * cq12 * cq3 * cq4 * cq6 * cq7 * sq5 * ( clx * mL + 7.735484874E-3 ) +
      gy * cq2 * cq3 * cq4 * cq6 * cq7 * sq1 * sq5 * ( clx * mL + 7.735484874E-3 ) -
      gx * cq12 * cq3 * cq4 * cq6 * sq5 * sq7 * ( cly * mL - 3.127439544E-3 ) -
      gy * cq2 * cq3 * cq4 * cq6 * sq1 * sq5 * sq7 * ( cly * mL - 3.127439544E-3 );
  g[5] = gz * cq2 * cq4 * cq6 * ( -1.00241616695E-1 ) - gz * cq2 * cq4 * sq6 * 2.3526756935E-2 -
         gx * cq14 * cq6 * sq2 * 1.00241616695E-1 - gy * cq13 * cq6 * sq5 * 2.3526756935E-2 -
         gz * cq2 * cq5 * cq6 * sq4 * 2.3526756935E-2 - gx * cq14 * sq2 * sq6 * 2.3526756935E-2 +
         gx * cq3 * cq6 * sq1 * sq5 * 2.3526756935E-2 - gy * cq4 * cq6 * sq1 * sq2 * 1.00241616695E-1 +
         gy * cq1 * cq6 * sq3 * sq4 * 1.00241616695E-1 + gy * cq13 * sq5 * sq6 * 1.00241616695E-1 -
         gz * cq3 * cq6 * sq2 * sq4 * 1.00241616695E-1 + gz * cq2 * cq5 * sq4 * sq6 * 1.00241616695E-1 -
         gz * cq2 * cq4 * sq6 * ( clz * mL + 4.5305948634E-2 ) - gx * cq6 * sq1 * sq3 * sq4 * 1.00241616695E-1 -
         gx * cq3 * sq1 * sq5 * sq6 * 1.00241616695E-1 - gy * cq4 * sq1 * sq2 * sq6 * 2.3526756935E-2 +
         gy * cq1 * sq3 * sq4 * sq6 * 2.3526756935E-2 - gz * cq3 * sq2 * sq4 * sq6 * 2.3526756935E-2 -
         gz * cq6 * sq2 * sq3 * sq5 * 2.3526756935E-2 - gx * sq1 * sq3 * sq4 * sq6 * 2.3526756935E-2 +
         gz * sq2 * sq3 * sq5 * sq6 * 1.00241616695E-1 - gz * cq2 * cq4 * cq6 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
         gx * cq12 * cq3 * sq4 * sq6 * 2.3526756935E-2 + gx * cq12 * cq6 * sq3 * sq5 * 2.3526756935E-2 -
         gx * cq15 * cq6 * sq2 * sq4 * 2.3526756935E-2 + gx * cq4 * cq5 * cq6 * sq1 * sq3 * 2.3526756935E-2 +
         gy * cq2 * cq3 * cq6 * sq1 * sq4 * 1.00241616695E-1 + gy * cq14 * cq5 * sq3 * sq6 * 1.00241616695E-1 -
         gz * cq3 * cq4 * cq5 * sq2 * sq6 * 1.00241616695E-1 - gy * cq13 * cq6 * sq5 * ( clz * mL + 4.5305948634E-2 ) -
         gz * cq2 * cq5 * cq6 * sq4 * ( clz * mL + 4.5305948634E-2 ) - gx * cq12 * sq3 * sq5 * sq6 * 1.00241616695E-1 +
         gx * cq15 * sq2 * sq4 * sq6 * 1.00241616695E-1 - gx * cq4 * cq5 * sq1 * sq3 * sq6 * 1.00241616695E-1 +
         gy * cq2 * cq3 * sq1 * sq4 * sq6 * 2.3526756935E-2 + gy * cq2 * cq6 * sq1 * sq3 * sq5 * 2.3526756935E-2 -
         gy * cq5 * cq6 * sq1 * sq2 * sq4 * 2.3526756935E-2 - gx * cq14 * sq2 * sq6 * ( clz * mL + 4.5305948634E-2 ) +
         gx * cq3 * cq6 * sq1 * sq5 * ( clz * mL + 4.5305948634E-2 ) -
         gy * cq2 * sq1 * sq3 * sq5 * sq6 * 1.00241616695E-1 + gy * cq5 * sq1 * sq2 * sq4 * sq6 * 1.00241616695E-1 -
         gy * cq4 * sq1 * sq2 * sq6 * ( clz * mL + 4.5305948634E-2 ) +
         gy * cq1 * sq3 * sq4 * sq6 * ( clz * mL + 4.5305948634E-2 ) -
         gz * cq3 * sq2 * sq4 * sq6 * ( clz * mL + 4.5305948634E-2 ) -
         gz * cq6 * sq2 * sq3 * sq5 * ( clz * mL + 4.5305948634E-2 ) -
         gx * sq1 * sq3 * sq4 * sq6 * ( clz * mL + 4.5305948634E-2 ) -
         gx * cq14 * cq6 * sq2 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
         gy * cq4 * cq6 * sq1 * sq2 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
         gy * cq1 * cq6 * sq3 * sq4 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
         gy * cq13 * sq5 * sq6 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
         gz * cq3 * cq6 * sq2 * sq4 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
         gz * cq2 * cq5 * sq4 * sq6 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
         gz * cq2 * cq4 * cq6 * sq7 * ( cly * mL - 3.127439544E-3 ) -
         gx * cq6 * sq1 * sq3 * sq4 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
         gx * cq3 * sq1 * sq5 * sq6 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
         gz * sq2 * sq3 * sq5 * sq6 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
         gz * cq2 * cq4 * cq6 * cq7 * ( clx * mL + 7.735484874E-3 ) + gx * cq12 * cq3 * cq6 * sq4 * 1.00241616695E-1 -
         gy * cq14 * cq5 * cq6 * sq3 * 2.3526756935E-2 + gz * cq3 * cq4 * cq5 * cq6 * sq2 * 2.3526756935E-2 +
         gy * cq2 * cq3 * sq1 * sq4 * sq6 * ( clz * mL + 4.5305948634E-2 ) +
         gy * cq2 * cq6 * sq1 * sq3 * sq5 * ( clz * mL + 4.5305948634E-2 ) -
         gy * cq5 * cq6 * sq1 * sq2 * sq4 * ( clz * mL + 4.5305948634E-2 ) +
         gx * cq12 * cq3 * cq6 * sq4 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
         gy * cq2 * cq3 * cq6 * sq1 * sq4 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
         gy * cq14 * cq5 * sq3 * sq6 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
         gz * cq3 * cq4 * cq5 * sq2 * sq6 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
         gx * cq12 * sq3 * sq5 * sq6 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
         gx * cq15 * sq2 * sq4 * sq6 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
         gx * cq4 * cq5 * sq1 * sq3 * sq6 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
         gx * cq14 * cq6 * sq2 * sq7 * ( cly * mL - 3.127439544E-3 ) -
         gy * cq2 * sq1 * sq3 * sq5 * sq6 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
         gy * cq5 * sq1 * sq2 * sq4 * sq6 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
         gy * cq4 * cq6 * sq1 * sq2 * sq7 * ( cly * mL - 3.127439544E-3 ) -
         gy * cq1 * cq6 * sq3 * sq4 * sq7 * ( cly * mL - 3.127439544E-3 ) -
         gy * cq13 * sq5 * sq6 * sq7 * ( cly * mL - 3.127439544E-3 ) +
         gz * cq3 * cq6 * sq2 * sq4 * sq7 * ( cly * mL - 3.127439544E-3 ) -
         gz * cq2 * cq5 * sq4 * sq6 * sq7 * ( cly * mL - 3.127439544E-3 ) +
         gx * cq6 * sq1 * sq3 * sq4 * sq7 * ( cly * mL - 3.127439544E-3 ) +
         gx * cq3 * sq1 * sq5 * sq6 * sq7 * ( cly * mL - 3.127439544E-3 ) -
         gz * sq2 * sq3 * sq5 * sq6 * sq7 * ( cly * mL - 3.127439544E-3 ) -
         gx * cq14 * cq6 * cq7 * sq2 * ( clx * mL + 7.735484874E-3 ) -
         gx * cq12 * cq3 * cq4 * cq5 * cq6 * 2.3526756935E-2 -
         gy * cq4 * cq6 * cq7 * sq1 * sq2 * ( clx * mL + 7.735484874E-3 ) +
         gy * cq1 * cq6 * cq7 * sq3 * sq4 * ( clx * mL + 7.735484874E-3 ) +
         gy * cq13 * cq7 * sq5 * sq6 * ( clx * mL + 7.735484874E-3 ) -
         gz * cq3 * cq6 * cq7 * sq2 * sq4 * ( clx * mL + 7.735484874E-3 ) +
         gz * cq2 * cq5 * cq7 * sq4 * sq6 * ( clx * mL + 7.735484874E-3 ) +
         gx * cq12 * cq3 * cq4 * cq5 * sq6 * 1.00241616695E-1 -
         gy * cq2 * cq3 * cq4 * cq5 * cq6 * sq1 * 2.3526756935E-2 -
         gx * cq6 * cq7 * sq1 * sq3 * sq4 * ( clx * mL + 7.735484874E-3 ) -
         gx * cq3 * cq7 * sq1 * sq5 * sq6 * ( clx * mL + 7.735484874E-3 ) +
         gy * cq2 * cq3 * cq4 * cq5 * sq1 * sq6 * 1.00241616695E-1 -
         gy * cq14 * cq5 * cq6 * sq3 * ( clz * mL + 4.5305948634E-2 ) +
         gz * cq3 * cq4 * cq5 * cq6 * sq2 * ( clz * mL + 4.5305948634E-2 ) +
         gz * cq7 * sq2 * sq3 * sq5 * sq6 * ( clx * mL + 7.735484874E-3 ) +
         gx * cq12 * cq3 * sq4 * sq6 * ( clz * mL + 4.5305948634E-2 ) +
         gx * cq12 * cq6 * sq3 * sq5 * ( clz * mL + 4.5305948634E-2 ) -
         gx * cq15 * cq6 * sq2 * sq4 * ( clz * mL + 4.5305948634E-2 ) +
         gx * cq4 * cq5 * cq6 * sq1 * sq3 * ( clz * mL + 4.5305948634E-2 ) +
         gy * cq2 * cq3 * cq4 * cq5 * sq1 * sq6 * ( mL + 7.35522E-1 ) * ( 0.088 ) -
         gx * cq12 * cq3 * cq6 * sq4 * sq7 * ( cly * mL - 3.127439544E-3 ) -
         gy * cq2 * cq3 * cq6 * sq1 * sq4 * sq7 * ( cly * mL - 3.127439544E-3 ) -
         gy * cq14 * cq5 * sq3 * sq6 * sq7 * ( cly * mL - 3.127439544E-3 ) +
         gz * cq3 * cq4 * cq5 * sq2 * sq6 * sq7 * ( cly * mL - 3.127439544E-3 ) +
         gx * cq12 * sq3 * sq5 * sq6 * sq7 * ( cly * mL - 3.127439544E-3 ) -
         gx * cq15 * sq2 * sq4 * sq6 * sq7 * ( cly * mL - 3.127439544E-3 ) +
         gx * cq4 * cq5 * sq1 * sq3 * sq6 * sq7 * ( cly * mL - 3.127439544E-3 ) +
         gy * cq2 * sq1 * sq3 * sq5 * sq6 * sq7 * ( cly * mL - 3.127439544E-3 ) -
         gy * cq5 * sq1 * sq2 * sq4 * sq6 * sq7 * ( cly * mL - 3.127439544E-3 ) +
         gx * cq12 * cq3 * cq6 * cq7 * sq4 * ( clx * mL + 7.735484874E-3 ) +
         gy * cq2 * cq3 * cq6 * cq7 * sq1 * sq4 * ( clx * mL + 7.735484874E-3 ) +
         gy * cq14 * cq5 * cq7 * sq3 * sq6 * ( clx * mL + 7.735484874E-3 ) -
         gz * cq3 * cq4 * cq5 * cq7 * sq2 * sq6 * ( clx * mL + 7.735484874E-3 ) -
         gx * cq12 * cq3 * cq4 * cq5 * cq6 * ( clz * mL + 4.5305948634E-2 ) -
         gx * cq12 * cq7 * sq3 * sq5 * sq6 * ( clx * mL + 7.735484874E-3 ) +
         gx * cq15 * cq7 * sq2 * sq4 * sq6 * ( clx * mL + 7.735484874E-3 ) -
         gx * cq4 * cq5 * cq7 * sq1 * sq3 * sq6 * ( clx * mL + 7.735484874E-3 ) -
         gy * cq2 * cq3 * cq4 * cq5 * cq6 * sq1 * ( clz * mL + 4.5305948634E-2 ) -
         gy * cq2 * cq7 * sq1 * sq3 * sq5 * sq6 * ( clx * mL + 7.735484874E-3 ) +
         gy * cq5 * cq7 * sq1 * sq2 * sq4 * sq6 * ( clx * mL + 7.735484874E-3 ) +
         gx * cq12 * cq3 * cq4 * cq5 * sq6 * ( mL + 7.35522E-1 ) * ( 0.088 ) +
         gx * cq12 * cq3 * cq4 * cq5 * cq7 * sq6 * ( clx * mL + 7.735484874E-3 ) +
         gy * cq2 * cq3 * cq4 * cq5 * cq7 * sq1 * sq6 * ( clx * mL + 7.735484874E-3 ) -
         gx * cq12 * cq3 * cq4 * cq5 * sq6 * sq7 * ( cly * mL - 3.127439544E-3 ) -
         gy * cq2 * cq3 * cq4 * cq5 * sq1 * sq6 * sq7 * ( cly * mL - 3.127439544E-3 );
  g[6] = -gy * cq13 * cq5 * sq7 * ( cly * mL - 3.127439544E-3 ) +
         gz * cq2 * cq4 * cq7 * sq6 * ( cly * mL - 3.127439544E-3 ) +
         gx * cq3 * cq5 * sq1 * sq7 * ( cly * mL - 3.127439544E-3 ) -
         gz * cq5 * sq2 * sq3 * sq7 * ( cly * mL - 3.127439544E-3 ) +
         gz * cq2 * sq4 * sq5 * sq7 * ( cly * mL - 3.127439544E-3 ) +
         gy * cq13 * cq5 * cq7 * ( clx * mL + 7.735484874E-3 ) -
         gx * cq3 * cq5 * cq7 * sq1 * ( clx * mL + 7.735484874E-3 ) +
         gz * cq5 * cq7 * sq2 * sq3 * ( clx * mL + 7.735484874E-3 ) -
         gz * cq2 * cq7 * sq4 * sq5 * ( clx * mL + 7.735484874E-3 ) +
         gz * cq2 * cq4 * sq6 * sq7 * ( clx * mL + 7.735484874E-3 ) +
         gx * sq1 * sq3 * sq4 * sq6 * sq7 * ( clx * mL + 7.735484874E-3 ) +
         gy * cq13 * cq6 * cq7 * sq5 * ( cly * mL - 3.127439544E-3 ) +
         gz * cq2 * cq5 * cq6 * cq7 * sq4 * ( cly * mL - 3.127439544E-3 ) +
         gx * cq12 * cq5 * sq3 * sq7 * ( cly * mL - 3.127439544E-3 ) +
         gx * cq14 * cq7 * sq2 * sq6 * ( cly * mL - 3.127439544E-3 ) -
         gx * cq3 * cq6 * cq7 * sq1 * sq5 * ( cly * mL - 3.127439544E-3 ) +
         gy * cq2 * cq5 * sq1 * sq3 * sq7 * ( cly * mL - 3.127439544E-3 ) +
         gy * cq14 * sq3 * sq5 * sq7 * ( cly * mL - 3.127439544E-3 ) +
         gy * cq4 * cq7 * sq1 * sq2 * sq6 * ( cly * mL - 3.127439544E-3 ) -
         gy * cq1 * cq7 * sq3 * sq4 * sq6 * ( cly * mL - 3.127439544E-3 ) -
         gz * cq3 * cq4 * sq2 * sq5 * sq7 * ( cly * mL - 3.127439544E-3 ) +
         gz * cq3 * cq7 * sq2 * sq4 * sq6 * ( cly * mL - 3.127439544E-3 ) +
         gz * cq6 * cq7 * sq2 * sq3 * sq5 * ( cly * mL - 3.127439544E-3 ) +
         gx * cq1 * sq2 * sq4 * sq5 * sq7 * ( cly * mL - 3.127439544E-3 ) -
         gx * cq4 * sq1 * sq3 * sq5 * sq7 * ( cly * mL - 3.127439544E-3 ) +
         gx * cq7 * sq1 * sq3 * sq4 * sq6 * ( cly * mL - 3.127439544E-3 ) +
         gy * sq1 * sq2 * sq4 * sq5 * sq7 * ( cly * mL - 3.127439544E-3 ) -
         gx * cq12 * cq5 * cq7 * sq3 * ( clx * mL + 7.735484874E-3 ) -
         gy * cq2 * cq5 * cq7 * sq1 * sq3 * ( clx * mL + 7.735484874E-3 ) -
         gy * cq14 * cq7 * sq3 * sq5 * ( clx * mL + 7.735484874E-3 ) +
         gy * cq13 * cq6 * sq5 * sq7 * ( clx * mL + 7.735484874E-3 ) +
         gz * cq3 * cq4 * cq7 * sq2 * sq5 * ( clx * mL + 7.735484874E-3 ) +
         gz * cq2 * cq5 * cq6 * sq4 * sq7 * ( clx * mL + 7.735484874E-3 ) -
         gx * cq1 * cq7 * sq2 * sq4 * sq5 * ( clx * mL + 7.735484874E-3 ) +
         gx * cq14 * sq2 * sq6 * sq7 * ( clx * mL + 7.735484874E-3 ) +
         gx * cq4 * cq7 * sq1 * sq3 * sq5 * ( clx * mL + 7.735484874E-3 ) -
         gx * cq3 * cq6 * sq1 * sq5 * sq7 * ( clx * mL + 7.735484874E-3 ) -
         gy * cq7 * sq1 * sq2 * sq4 * sq5 * ( clx * mL + 7.735484874E-3 ) +
         gy * cq4 * sq1 * sq2 * sq6 * sq7 * ( clx * mL + 7.735484874E-3 ) -
         gy * cq1 * sq3 * sq4 * sq6 * sq7 * ( clx * mL + 7.735484874E-3 ) +
         gz * cq3 * sq2 * sq4 * sq6 * sq7 * ( clx * mL + 7.735484874E-3 ) +
         gz * cq6 * sq2 * sq3 * sq5 * sq7 * ( clx * mL + 7.735484874E-3 ) +
         gy * cq14 * cq5 * cq6 * cq7 * sq3 * ( cly * mL - 3.127439544E-3 ) -
         gz * cq3 * cq4 * cq5 * cq6 * cq7 * sq2 * ( cly * mL - 3.127439544E-3 ) +
         gx * cq12 * cq3 * cq4 * sq5 * sq7 * ( cly * mL - 3.127439544E-3 ) -
         gx * cq12 * cq3 * cq7 * sq4 * sq6 * ( cly * mL - 3.127439544E-3 ) -
         gx * cq12 * cq6 * cq7 * sq3 * sq5 * ( cly * mL - 3.127439544E-3 ) +
         gx * cq15 * cq6 * cq7 * sq2 * sq4 * ( cly * mL - 3.127439544E-3 ) -
         gx * cq4 * cq5 * cq6 * cq7 * sq1 * sq3 * ( cly * mL - 3.127439544E-3 ) +
         gy * cq2 * cq3 * cq4 * sq1 * sq5 * sq7 * ( cly * mL - 3.127439544E-3 ) -
         gy * cq2 * cq3 * cq7 * sq1 * sq4 * sq6 * ( cly * mL - 3.127439544E-3 ) -
         gy * cq2 * cq6 * cq7 * sq1 * sq3 * sq5 * ( cly * mL - 3.127439544E-3 ) +
         gy * cq5 * cq6 * cq7 * sq1 * sq2 * sq4 * ( cly * mL - 3.127439544E-3 ) -
         gx * cq12 * cq3 * cq4 * cq7 * sq5 * ( clx * mL + 7.735484874E-3 ) -
         gy * cq2 * cq3 * cq4 * cq7 * sq1 * sq5 * ( clx * mL + 7.735484874E-3 ) +
         gy * cq14 * cq5 * cq6 * sq3 * sq7 * ( clx * mL + 7.735484874E-3 ) -
         gz * cq3 * cq4 * cq5 * cq6 * sq2 * sq7 * ( clx * mL + 7.735484874E-3 ) -
         gx * cq12 * cq3 * sq4 * sq6 * sq7 * ( clx * mL + 7.735484874E-3 ) -
         gx * cq12 * cq6 * sq3 * sq5 * sq7 * ( clx * mL + 7.735484874E-3 ) +
         gx * cq15 * cq6 * sq2 * sq4 * sq7 * ( clx * mL + 7.735484874E-3 ) -
         gx * cq4 * cq5 * cq6 * sq1 * sq3 * sq7 * ( clx * mL + 7.735484874E-3 ) -
         gy * cq2 * cq3 * sq1 * sq4 * sq6 * sq7 * ( clx * mL + 7.735484874E-3 ) -
         gy * cq2 * cq6 * sq1 * sq3 * sq5 * sq7 * ( clx * mL + 7.735484874E-3 ) +
         gy * cq5 * cq6 * sq1 * sq2 * sq4 * sq7 * ( clx * mL + 7.735484874E-3 ) +
         gx * cq12 * cq3 * cq4 * cq5 * cq6 * sq7 * ( clx * mL + 7.735484874E-3 ) +
         gy * cq2 * cq3 * cq4 * cq5 * cq6 * sq1 * sq7 * ( clx * mL + 7.735484874E-3 ) +
         gx * cq12 * cq3 * cq4 * cq5 * cq6 * cq7 * ( cly * mL - 3.127439544E-3 ) +
         gy * cq2 * cq3 * cq4 * cq5 * cq6 * cq7 * sq1 * ( cly * mL - 3.127439544E-3 );

  return g;
}

} // namespace franka_model
