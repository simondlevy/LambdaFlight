{--
  EKF algorithm
 
  Copyright (C) 2024 Simon D. Levy
 
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, in version 3.
 
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http:--www.gnu.org/licenses/>.
--} 

{-# LANGUAGE DataKinds        #-}
{-# LANGUAGE RebindableSyntax #-}

module Main where

import Language.Copilot hiding(atan2, sum, (!!))
import Copilot.Compile.C99

import Linear.Matrix hiding(transpose)
import Data.List hiding(sum) -- gives us transpose

import Utils

-- Initial variances, uncertain of position, but know we're stationary and
-- roughly flat
stdev_initial_position_z         = 1.0  :: SFloat
stdev_initial_velocity           = 0.01 :: SFloat
stdev_initial_attituderoll_pitch = 0.01 :: SFloat
stdev_initial_attitude_yaw       = 0.01 :: SFloat

proc_noise_acc_xy = 0.5 :: SFloat
proc_noise_acc_z = 1.0 :: SFloat
proc_noise_vel = 0 :: SFloat
proc_noise_pos = 0 :: SFloat
proc_noise_att = 0 :: SFloat
meas_noise_gyro_roll_pitch = 0.1 :: SFloat -- radians per second
meas_noise_gyro_roll_yaw = 0.1   :: SFloat -- radians per second

--The reversion of pitch and roll to zero
rollpitch_zero_reversion = 0.001 :: SFloat

mss_to_gs = 9.81 :: SFloat

rad_to_deg = 180 / pi :: SFloat

-- Small number epsilon, to prevent dividing by zero
eps = 1e-6 :: SFloat

-- Bounds on the covariance, these shouldn't be hit, but sometimes are... why?
max_covariance = 100 :: SFloat
min_covariance = 1e-6 :: SFloat

------------------------------------------------------------------------------

type EkfMode = SInt8

mode_init              = 0 :: EkfMode
mode_predict           = 1 :: EkfMode
mode_finalize          = 2 :: EkfMode
mode_get_state         = 3 :: EkfMode
mode_update_with_gyro  = 4 :: EkfMode
mode_update_with_accel = 5 :: EkfMode
mode_update_with_range = 6 :: EkfMode
mode_update_with_flow  = 7 :: EkfMode

------------------------------------------------------------------------------


ekfMode :: EkfMode
ekfMode = extern "stream_ekfMode" Nothing

nowMsec :: SInt32
nowMsec = extern "stream_nowMsec" Nothing

nextPredictionMsec :: SInt32
nextPredictionMsec = extern "stream_nextPredictionMsec" Nothing

isFlying :: SBool
isFlying = extern "stream_isFlying" Nothing

gx :: SFloat
gx = extern "stream_gx" Nothing

gyroY :: SFloat
gyroY = extern "stream_gyroY" Nothing

gyroZ :: SFloat
gyroZ = extern "stream_gyroZ" Nothing

accelX :: SFloat
accelX = extern "stream_accelX" Nothing

accelY :: SFloat
accelY = extern "stream_accelY" Nothing

accelZ :: SFloat
accelZ = extern "stream_accelZ" Nothing

------------------------------------------------------------------------------

data Quaternion = Quaternion {
    qqw :: SFloat
  , qqx :: SFloat
  , qqy :: SFloat
  , qqz :: SFloat
}

------------------------------------------------------------------------------

data EkfState = EkfState {
    ezz :: SFloat
  , edx :: SFloat
  , edy :: SFloat
  , edz :: SFloat
  , ee0 :: SFloat
  , ee1 :: SFloat
  , ee2 :: SFloat
}

------------------------------------------------------------------------------

data VehicleState = VehicleState {
    vz :: SFloat
  , vdx :: SFloat
  , vdy :: SFloat
  , vdz :: SFloat
  , phi :: SFloat
  , theta :: SFloat
  , psi :: SFloat
}

------------------------------------------------------------------------------

data Axis3 = Axis3 {
    x :: SFloat
  , y :: SFloat
  , z :: SFloat
}

------------------------------------------------------------------------------

data SubSampler = SubSampler { 
    sample :: Axis3
  , sum    :: Axis3
  , count  :: SInt32
}

------------------------------------------------------------------------------

data Ekf = Ekf {
    p :: Matrix
  , r :: Axis3
  , quat :: Quaternion
  , ekfState :: EkfState
  , gyroSubSampler :: SubSampler
  , accelSubSampler :: SubSampler
  , isUpdated :: SBool
  , lastPredictionMsec :: SInt32
  , lastProcessNoiseUpdateMsec :: SInt32
}

------------------------------------------------------------------------------

type Vector = [SFloat]

type Matrix = [Vector]

type Index = Int

------------------------------------------------------------------------------

aLowerRight :: SFloat -> SFloat -> SFloat ->
  (SFloat, SFloat, SFloat, SFloat, SFloat, SFloat, SFloat, SFloat, SFloat)

aLowerRight e0 e1 e2 =  (e00, e01, e02, e10, e11, e12, e20, e21, e22) where

  e00 =  1 - e1*e1/2 - e2*e2/2
  e01 =  e2 + e0*e1/2
  e02 = (-e1) + e0*e2/2

  e10 =  (-e2) + e0*e1/2
  e11 = 1 - e0*e0/2 - e2*e2/2
  e12 = e0 + e1*e2/2

  e20 = e1 + e0*e2/2
  e21 = (-e0) + e1*e2/2
  e22 = 1 - e0*e0/2 - e1*e1/2

------------------------------------------------------------------------------

updateQuatValue :: SBool -> SBool -> SFloat -> SFloat -> SFloat -> SFloat -> SFloat

updateQuatValue shouldPredict isErrorSufficient tmpq norm init curr =
  if ekfMode == mode_init then init
  else if shouldPredict then tmpq / norm
  else if isErrorSufficient then tmpq / norm
  else curr

------------------------------------------------------------------------------

updateRotationValue :: SFloat -> SFloat -> SFloat -> SFloat

updateRotationValue init curr final = 

  if ekfMode == mode_init then init
  else if ekfMode == mode_finalize then final
  else curr

------------------------------------------------------------------------------

updateEkfValue :: SBool -> SFloat -> SFloat -> SFloat

updateEkfValue shouldPredict curr predicted = 

  if ekfMode == mode_init then 0 
  else if shouldPredict then predicted
  else curr

------------------------------------------------------------------------------

rotateQuat :: SFloat -> SFloat -> SFloat

rotateQuat val initVal = val' where

  keep = 1 - rollpitch_zero_reversion

  val' = (val * (if isFlying then 1 else keep)) +
      (if isFlying then 0 else rollpitch_zero_reversion * initVal)

------------------------------------------------------------------------------

isErrorLarge :: SFloat -> SBool 
isErrorLarge v = abs v > 0.1e-3

isErrorInBounds :: SFloat -> SBool 
isErrorInBounds v = abs v < 10

------------------------------------------------------------------------------

subSamplerInit = SubSampler sample sum 0 where

  sum = Axis3 0 0 0

  sample = Axis3 0 0 0

------------------------------------------------------------------------------

ekfInit :: Ekf

ekfInit = Ekf p r q s g a false nowMsec nowMsec where 

  qq = sqr stdev_initial_position_z 
  dd = sqr stdev_initial_velocity 
  ee = sqr stdev_initial_attituderoll_pitch
  rr = sqr stdev_initial_attitude_yaw

  p     =  [--   z   dx  dy  dz  e0  e1  e2
              [qq,  0,  0,  0,  0,  0,  0], -- z
              [0,  dd,  0,  0,  0,  0,  0], -- dx
              [0,  0,  dd,  0,  0,  0,  0], -- dy
              [0,  0,  0,  dd,  0,  0,  0], -- dz
              [0,  0,  0,  0,  ee,  0,  0], -- e0
              [0,  0,  0,  0,  0,  ee,  0], -- e1
              [0,  0,  0,  0,  0,  0,  rr]  -- e2
          ] 

  r = Axis3 0 0 1

  q = Quaternion 1 0 0 0

  s = EkfState 0 0 0 0 0 0 0

  g = subSamplerInit

  a = subSamplerInit

------------------------------------------------------------------------------

updateSum subSampler accessor shouldFinalize conversionFactor count = 

  if shouldFinalize 
  then (accessor (sum subSampler)) * conversionFactor / count
  else (accessor (sample subSampler))


subSamplerFinalize :: SBool -> SubSampler -> SFloat -> SubSampler

subSamplerFinalize shouldPredict subSampler conversionFactor = subSampler' where

  count' = (unsafeCast (count subSampler)) :: SFloat

  shouldFinalize = shouldPredict &&  count' > 0

  x' = updateSum subSampler x shouldFinalize conversionFactor count'
  y' = updateSum subSampler y shouldFinalize conversionFactor count'
  z' = updateSum subSampler z shouldFinalize conversionFactor count'

  subSampler' = SubSampler (Axis3 x' y' z') (Axis3 0 0 0) 0
 
------------------------------------------------------------------------------

(!) :: Matrix -> (Index, Index) -> SFloat
a ! (i, j) = (a !! i) !! j

------------------------------------------------------------------------------

-- Enforce symmetry of covariance matrix, ensuring values stay bounded

updateCovarianceMatrix :: Matrix -> SBool -> Matrix

updateCovarianceMatrix p shouldUpdate = p' where 

  d j = let pval = p!(j,j) in 
    if shouldUpdate && pval < min_covariance then min_covariance else pval

  o i j = let pval = p!(i,j) in 
    if shouldUpdate && pval > max_covariance then max_covariance else pval

  p' = [ 
         [d 0,   o 0 1, o 0 2, o 0 3, o 0 4, o 0 5, o 0 6],
         [o 1 0, d 1,   o 1 2, o 1 3, o 1 4, o 1 5, o 1 6],
         [o 2 0, o 2 1, d 2,   o 2 3, o 2 4, o 2 5, o 2 6],
         [o 3 0, o 3 1, o 3 2, d 3,   o 3 4, o 3 5, o 3 6],
         [o 4 0, o 4 1, o 4 2, o 4 3, d 4,   o 4 5, o 4 6],
         [o 5 0, o 5 1, o 5 2, o 5 3, o 5 4, d 5,   o 5 6],
         [o 6 0, o 6 1, o 6 2, o 6 3, o 6 4, o 6 5, d 6]
       ]


------------------------------------------------------------------------------

addNoiseDiagonal :: Matrix -> Vector -> SBool -> Matrix

addNoiseDiagonal p diag shouldUpdate = p' where


  d j = p!(j,j) + if shouldUpdate then diag!!j else 0

  p' = [ 
         [d 0,     p!(0,1), p!(0,2), p!(0,3), p!(0,4), p!(0,5), p!(0,6)],
         [p!(1,0), d 1,     p!(1,2), p!(1,3), p!(1,4), p!(1,5), p!(1,6)],
         [p!(2,0), p!(2,1), d 2,     p!(2,3), p!(2,4), p!(2,5), p!(2,6)],
         [p!(3,0), p!(3,1), p!(3,2), d 3,     p!(3,4), p!(3,5), p!(3,6)],
         [p!(4,0), p!(4,1), p!(4,2), p!(4,3), d 4,     p!(4,5), p!(4,6)],
         [p!(5,0), p!(5,1), p!(5,2), p!(5,3), p!(5,4), d 5,     p!(5,6)],
         [p!(6,0), p!(6,1), p!(6,2), p!(6,3), p!(6,4), p!(6,5), d 6]
       ]


------------------------------------------------------------------------------

getDt :: SInt32 -> SInt32 -> SFloat

getDt msec1 msec2 = (unsafeCast (msec1 - msec2)) / 1000

------------------------------------------------------------------------------

ekfPredict :: Ekf -> Ekf

ekfPredict ekf = ekf where

  shouldPredict = nowMsec >= nextPredictionMsec

  gyroSubSampler' = subSamplerFinalize shouldPredict (gyroSubSampler ekf) rad_to_deg

  accelSubSampler' = subSamplerFinalize shouldPredict (accelSubSampler ekf) mss_to_gs

  gyro = sample gyroSubSampler'

  accel = sample accelSubSampler'

  ekfs = ekfState ekf

  dt = getDt nowMsec (lastPredictionMsec ekf)

  dt2 = dt * dt

  -- Position updates in the body frame (will be rotated to inertial frame)
  -- thrust can only be produced in the body's Z direction
  dx = (edx ekfs) * dt + if isFlying then 0 else (x accel) * dt2 / 2
  dy = (edy ekfs) * dt + if isFlying then 0 else (y accel) * dt2 / 2
  dz = (edz ekfs) * dt + (z accel) * dt2 / 2 

  -- Keep previous time step's state for the update
  tmpSDX = (edx ekfs)
  tmpSDY = (edx ekfs)
  tmpSDZ = (edz ekfs)

  accelx = if isFlying then 0 else x accel
  accely = if isFlying then 0 else y accel

  gyrox = x gyro
  gyroy = y gyro
  gyroz = z gyro

  -- Attitude update (rotate by gyroscope), we do this in quaternions.
  -- This is the gyroscope angular velocity integrated over the sample period.
  dtwx = dt * gyrox
  dtwy = dt * gyroy
  dtwz = dt * gyroz

  -- Compute the quaternion values in [w,x,y,z] order
  angle = sqrt (dtwx*dtwx + dtwy*dtwy + dtwz*dtwz) + eps
  ca = cos $ angle / 2
  sa = sin $ angle / 2
  dqw = ca
  dqx = sa * dtwx / angle
  dqy = sa * dtwy / angle
  dqz = sa * dtwz / angle

  -- Rotate the quad's attitude by the delta quaternion vector computed above

  quat' = (quat ekf)
  qw = qqw quat'
  qx = qqx quat'
  qy = qqy quat'
  qz = qqz quat'

  tmpq0 = rotateQuat (dqw*qw - dqx*qx - dqy*qy - dqz*qz) 1
  tmpq1 = rotateQuat (dqx*qw + dqw*qx + dqz*qy - dqy*qz) 0
  tmpq2 = rotateQuat (dqy*qw - dqz*qx + dqw*qy + dqx*qz) 0
  tmpq3 = rotateQuat (dqz*qw + dqy*qx - dqx*qy + dqw*qz) 0

  -- Normalize and store the result
  norm = sqrt (tmpq0*tmpq0 + tmpq1*tmpq1 + tmpq2*tmpq2 + tmpq3*tmpq3) + eps

  -- Process noise is added after the return from the prediction step

  -- ====== PREDICTION STEP ======
  -- The prediction depends on whether we're on the ground, or in flight.
  -- When flying, the accelerometer directly measures thrust (hence is useless
  -- to estimate body angle while flying)

  r' = (r ekf)
  rx = x r'
  ry = y r'
  rz = z r'

  -- altitude update
  z' = (ezz ekfs) + 
    (if shouldPredict 
     then rx * dx + ry * dy +  rz * dz - mss_to_gs * dt2 / 2
     else 0)

   -- body-velocity update: accelerometers - gyros cross velocity
   -- - gravity in body frame

  dx' = (edx ekfs) + 
     (if shouldPredict 
      then dt * (accelx + gyroz * tmpSDY - gyroy * tmpSDZ - mss_to_gs * rx) 
      else 0)

  dy' = (edy ekfs) + 
     (if shouldPredict 
      then dt * (accely - gyroz * tmpSDX + gyrox * tmpSDZ - mss_to_gs * ry)
      else 0)

  dz' = (edz ekfs) + 
     (if shouldPredict 
      then dt * ((z accel) + gyroy * tmpSDX - gyrox * tmpSDY - mss_to_gs * rz) 
      else 0)


  qw' = if shouldPredict then tmpq0/norm else qw
  qx' = if shouldPredict then tmpq1/norm else qx 
  qy' = if shouldPredict then tmpq2/norm else qy
  qz' = if shouldPredict then tmpq3/norm else qz

  isUpdated' = if shouldPredict then true else (isUpdated ekf)

  lastPredictionMsec' = if shouldPredict then nowMsec else (lastPredictionMsec ekf)

  -- ====== COVARIANCE UPDATE ======

  e0 = (x gyro) * dt / 2
  e1 = (y gyro) * dt / 2
  e2 = (z gyro) * dt / 2

  (e00, e01, e02, e10, e11, e12, e20, e21, e22) = aLowerRight e0 e1 e2

  -- altitude from body-frame velocity
  zdx  = rx * dt
  zdy  = ry * dt
  zdz  = rz * dt

  -- altitude from attitude error
  ze0  = ((edy ekfs) * rz - (edz ekfs) * ry) * dt
  ze1  = (- (edx ekfs) * rz + (edz ekfs) * rx) * dt
  ze2  = ((edx ekfs) * ry - (edy ekfs) * rx) * dt

  -- body-frame velocity from body-frame velocity
  dxdx  = 1 --drag negligible
  dydx =  -gyroz * dt
  dzdx  = gyroy * dt

  dxdy  = gyroz * dt
  dydy  = 1 --drag negligible
  dzdy  = gyrox * dt

  dxdz =  gyroy * dt
  dydz  = gyrox * dt
  dzdz  = 1 --drag negligible

  -- body-frame velocity from attitude error
  dxe0  = 0
  dye0  = -mss_to_gs * (z r') * dt
  dze0  = mss_to_gs * (y r') * dt

  dxe1  = mss_to_gs * (z r') * dt
  dye1  = 0
  dze1  = -mss_to_gs * (x r') * dt

  dxe2  = -mss_to_gs * (y r') * dt
  dye2  = mss_to_gs * (x r') * dt
  dze2  = 0

  a =  [  --  z   dx   dy    dz    e1    e1    e2
             [0, zdx,  zdy,  zdz,  ze0,  ze1,  ze2],  -- z
             [0, dxdx, dxdy, dxdz, dxe0, dxe1, dxe2], -- dx
             [0, dydx, dydy, dydz, dye0, dye1, dye2], -- dy
             [0, dzdx, dzdy, dzdz, dze0, dze1, dze2], -- dz
             [0, 0,    0,    0,    e00,  e01,  e02],  -- e0
             [0, 0,    0,    0,    e10,  e11,  e12],  -- e1
             [0, 0,    0,    0,    e20,  e21,  e22]   -- e2
       ] 

  p' = a !*! (p ekf) !*! (transpose a)  -- P <- APA'

  dt' = getDt nowMsec  (lastProcessNoiseUpdateMsec ekf)
  isDtPositive = dt' > 0

  -- Add process noise 
  noise = [
            sqr (proc_noise_acc_z*dt'*dt' + proc_noise_vel*dt' + proc_noise_pos),
            sqr (proc_noise_acc_xy*dt' + proc_noise_vel),
            sqr (proc_noise_acc_xy*dt' + proc_noise_vel), 
            sqr (proc_noise_acc_z*dt' + proc_noise_vel),
            sqr (meas_noise_gyro_roll_pitch * dt' + proc_noise_att),
            sqr (meas_noise_gyro_roll_pitch * dt' + proc_noise_att),
            sqr (meas_noise_gyro_roll_yaw * dt' + proc_noise_att)
          ]

  p'' = updateCovarianceMatrix (addNoiseDiagonal p' noise isDtPositive) isDtPositive

------------------------------------------------------------------------------

ekfFinalize :: Ekf -> Ekf

ekfFinalize ekf = ekf where

  ekfs = ekfState ekf

  v0 = ee0 ekfs
  v1 = ee1 ekfs
  v2 = ee2 ekfs

  angle = sqrt (v0*v0 + v1*v1 + v2*v2) + eps
  ca = cos $ angle / 2
  sa = sin $ angle / 2

  dqw = ca
  dqx = sa * v0 / angle
  dqy = sa * v1 / angle
  dqz = sa * v2 / angle

  q = quat ekf
  qw = qqw q
  qx = qqx q
  qy = qqy q
  qz = qqz q

  -- Rotate the quad's attitude by the delta quaternion vector computed above
  tmpq0 = dqw * qw - dqx * qx - dqy * qy - dqz * qz
  tmpq1 = dqx * qw + dqw * qx + dqz * qy - dqy * qz
  tmpq2 = dqy * qw - dqz * qx + dqw * qy + dqx * qz
  tmpq3 = dqz * qw + dqy * qx - dqx * qy + dqw * qz

  -- Normalize and store the result
  norm = sqrt (tmpq0 * tmpq0 + tmpq1 * tmpq1 + tmpq2 * tmpq2 + tmpq3 * tmpq3) + eps

  isErrorSufficient  = 
                (isErrorLarge v0  || isErrorLarge v1  || isErrorLarge v2 ) &&
                isErrorInBounds v0  && isErrorInBounds v1  && isErrorInBounds v2

  qw' = if isErrorSufficient then tmpq0 / norm else qw
  qx' = if isErrorSufficient then tmpq1 / norm else qx
  qy' = if isErrorSufficient then tmpq2 / norm else qy
  qz' = if isErrorSufficient then tmpq3 / norm else qz

  -- Move attitude error into attitude if any of the angle errors are large
  -- enough
  (e00, e01, e02, e10, e11, e12, e20, e21, e22) = aLowerRight (v0/2) (v1/2) (v2/2)
  a =  [ --  z   dx  dy  dz  e0    e1     e2
            [1 , 0,  0,  0,  0,    0,     0], -- z
            [0,  1 , 0,  0,  0,    0,     0], -- dx
            [0,  0,  1 , 0,  0,    0,     0], -- dy
            [0,  0,  0,  1 , 0,    0,     0], -- dz
            [0,  0,  0,  0,  e00, e01,  e02], -- e0
            [0,  0,  0,  0,  e10, e11,  e12], -- e1
            [0,  0,  0,  0,  e20, e21,  e22]  -- e2
       ] 

------------------------------------------------------------------------------

step = (vz, vdx, vdy, vdz, phi, theta, psi) where

  vz = 0 :: SFloat
  vdx = 0 :: SFloat
  vdy = 0 :: SFloat
  vdz = 0 :: SFloat
  phi = 0 :: SFloat
  theta = 0 :: SFloat
  psi = 0 :: SFloat

  --------------------------------------------------------------------------

{--
  _qw = [1] ++ qw
  _qx = [1] ++ qx
  _qy = [1] ++ qy
  _qz = [1] ++ qz
 
  _rx = [0] ++ rx
  _ry = [0] ++ ry
  _rz = [0] ++ rz

  _zz = [0] ++ zz
  _dx = [0] ++ dx
  _dy = [0] ++ dy
  _dz = [0] ++ dz
  _e0 = [0] ++ e0
  _e1 = [0] ++ e1
  _e2 = [0] ++ e2

  _lastPredictionMsec = [0] ++ lastPredictionMsec
--}

------------------------------------------------------------------------------

spec = do

  let (vz, vdx, vdy, vdz, phi, theta, psi) = step

  trigger "setState" 
           (ekfMode == mode_get_state) 
           [arg vz, arg vdx, arg vdy, arg vdz, arg phi, arg theta, arg psi]

main = reify spec >>= 

  compileWith (CSettings "ekf_step" ".") "ekf"
