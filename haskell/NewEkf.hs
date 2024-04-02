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

import Language.Copilot hiding(atan2, sum)
import Copilot.Compile.C99

import Utils

-- Initial variances, uncertain of position, but know we're stationary and
-- roughly flat
stdev_initial_position_z         = 1.0  :: SFloat
stdev_initial_velocity           = 0.01 :: SFloat
stdev_initial_attituderoll_pitch = 0.01 :: SFloat
stdev_initial_attitude_yaw       = 0.01 :: SFloat

--The reversion of pitch and roll to zero
rollpitch_zero_reversion = 0.001 :: SFloat

gravity_magnitude = 9.81 :: SFloat

-- Small number epsilon, to prevent dividing by zero
eps = 1e-6 :: SFloat

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

type EkfMatrix = Array 7 (Array 7 SFloat)

pinit :: EkfMatrix

qq = sqr stdev_initial_position_z 
dd = sqr stdev_initial_velocity 
ee = sqr stdev_initial_attituderoll_pitch
rr = sqr stdev_initial_attitude_yaw

pinit =  array [--  z   dx  dy  dz  e0  e1  e2
             array [qq,  0,  0,  0,  0,  0,  0], -- z
             array [0,  dd,  0,  0,  0,  0,  0], -- dx
             array [0,  0,  dd,  0,  0,  0,  0], -- dy
             array [0,  0,  0,  dd,  0,  0,  0], -- dz
             array [0,  0,  0,  0,  ee,  0,  0], -- e0
             array [0,  0,  0,  0,  0,  ee,  0], -- e1
             array [0,  0,  0,  0,  0,  0,  rr]  -- e2
             ] 

------------------------------------------------------------------------------
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
  , e0 :: SFloat
  , e1 :: SFloat
  , e2 :: SFloat
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
    p :: EkfMatrix
  , r :: Axis3
  , quat :: Quaternion
  , ekfState :: EkfState
  , gyroSubSampler :: SubSampler
  , accelSubSampler :: SubSampler
  , isUpdated :: SBool
  , lastPredictionMsec :: SInt32
  , lastProcessedNoiseUpdateMsec :: SInt32
}

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

subSamplerInit = SubSampler sample sum count where

  sum = Axis3 0 0 0
  count = 0
  sample = Axis3 0 0 0

------------------------------------------------------------------------------

ekfInit :: Ekf

ekfInit = Ekf p r q s g a false 0 0 where 

  p = pinit

  r = Axis3 0 0 1

  q = Quaternion 1 0 0 0

  s = EkfState 0 0 0 0 0 0 0

  g = subSamplerInit

  a = subSamplerInit

------------------------------------------------------------------------------

step = (vz, vdx, vdy, vdz, phi, theta, psi) where

    shouldPredict = ekfMode == mode_predict && nowMsec >= nextPredictionMsec

    lastPredictionMsec = 
        if ekfMode == mode_init then nowMsec else _lastPredictionMsec

    dmsec = (unsafeCast $ nowMsec - lastPredictionMsec) :: SFloat

    dt = dmsec / 1000.0

    e0' = gyrox * dt / 2
    e1' = gyroy * dt / 2
    e2' = gyroz * dt / 2

    -- Altitude from body-frame velocity
    zdx = rx * dt
    zdy = ry * dt
    zdz = rz * dt

    -- Altitude from attitude error
    ze0 = (dy * rz - dz * ry) * dt
    ze1 = (-dx * rz + dz * rx) * dt
    ze2 = (dx * ry - dy * rx) * dt

    -- Body-frame velocity from body-frame velocity
    dxdx =  1 --drag negligible
    dydx = -(gyroz * dt)
    dzdx =  gyroy * dt

    dxdy =  gyroz * dt
    dydy =  1 --drag negligible
    dzdy = -(gyrox * dt)

    dxdz = -(gyroy * dt)
    dydz =  gyrox * dt
    dzdz =  1 --drag negligible

    -- Body-frame velocity from attitude error
    dxe0 =  0
    dye0 = -(gravity_magnitude * rz * dt)
    dze0 =  gravity_magnitude * ry * dt

    dxe1 =  gravity_magnitude * rz * dt
    dye1 =  0
    dze1 = -(gravity_magnitude * rx * dt)

    dxe2 = -(gravity_magnitude * ry * dt)
    dye2 =  gravity_magnitude * rx * dt
    dze2 =  0

    e0e0 =  1 - e1 * e1/2 - e2 * e2/2
    e0e1 =  e2 + e0 * e1/2
    e0e2 = -e1 + e0 * e2/2

    e1e0 = -e2 + e0 * e1/2
    e1e1 =  1 - e0 * e0/2 - e2 * e2/2
    e1e2 =  e0 + e1 * e2/2

    e2e0 =  e1 + e0 * e2/2
    e2e1 = -e0 + e1 * e2/2
    e2e2 = 1 - e0 * e0/2 - e1 * e1/2

    dt2 = dt * dt
 
    -- XXX need to compute these for real
    gyrox = 0
    gyroy = 0
    gyroz = 0
    accelx = 0
    accely = 0
    accelz = 0
    isErrorSufficient = ekfMode == mode_finalize && true

    -- Position updates in the body frame (will be rotated to inertial frame)
    -- thrust can only be produced in the body's Z direction
    dx' = dx * dt + (if isFlying then 0 else accelx * dt2 / 2)
    dy' = dy * dt + (if isFlying then 0 else accely * dt2 / 2)
    dz' = dz * dt + accelz * dt2 / 2 

    -- Keep previous time step's state for the update
    tmpSDX = dx
    tmpSDY = dy
    tmpSDZ = dz

    accelx' = if isFlying then 0 else accelx
    accely' = if isFlying then 0 else accely

    -- Attitude update (rotate by gyroscope), we do this in quaternions.
    -- This is the gyroscope angular velocity integrated over the sample period
    dtwx = dt * gyrox
    dtwy = dt * gyroy
    dtwz = dt * gyroz

    -- Compute the quaternion values in [w,x,y,z] order
    angle = sqrt (dtwx*dtwx + dtwy*dtwy + dtwz*dtwz) + eps
    ca = cos $ angle/2
    sa = sin $ angle/2
    dqw = ca
    dqx = sa * dtwx / angle
    dqy = sa * dtwy / angle
    dqz = sa * dtwz / angle

    -- Rotate the quad's attitude by the delta quaternion vector computed above
    tmpq0 = rotateQuat (dqw*_qw - dqx*_qx - dqy*_qy - dqz*_qz) 1
    tmpq1 = rotateQuat (dqx*_qw + dqw*_qx + dqz*_qy - dqy*_qz) 0
    tmpq2 = rotateQuat (dqy*_qw - dqz*_qx + dqw*_qy + dqx*_qz) 0
    tmpq3 = rotateQuat (dqz*_qw + dqy*_qx - dqx*_qy + dqw*_qz) 0

    -- Normalize and store the result
    norm = sqrt (tmpq0*tmpq0 + tmpq1*tmpq1 + tmpq2*tmpq2 + tmpq3*tmpq3) + eps

    -- Quaternion
    qw = updateQuatValue shouldPredict isErrorSufficient tmpq0 norm 1 _qw
    qx = updateQuatValue shouldPredict isErrorSufficient tmpq1 norm 0 _qx
    qy = updateQuatValue shouldPredict isErrorSufficient tmpq2 norm 0 _qy
    qz = updateQuatValue shouldPredict isErrorSufficient tmpq3 norm 0 _qz

    -- Rotation vector
    rx = updateRotationValue 0 _rx (2*_qx*_qz - 2*_qw*_qy)
    ry = updateRotationValue 0 _ry (2*_qy*_qz + 2*_qw*_qx)
    rz = updateRotationValue 1 _rz (_qw*_qw - _qx*_qx - _qy*_qy + _qz*_qz)

    -- EKF state
    zz = updateEkfValue shouldPredict _zz 0

    dx = 0 :: SFloat

    dy = 0 :: SFloat

    dz = 0 :: SFloat

    e0 = 0 :: SFloat

    e1 = 0 :: SFloat

    e2 = 0 :: SFloat

    vz = 0 :: SFloat
    vdx = 0 :: SFloat
    vdy = 0 :: SFloat
    vdz = 0 :: SFloat
    phi = 0 :: SFloat
    theta = 0 :: SFloat
    psi = 0 :: SFloat

    --------------------------------------------------------------------------

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

------------------------------------------------------------------------------

spec = do

  let (vz, vdx, vdy, vdz, phi, theta, psi) = step

  trigger "setState" 
           (ekfMode == mode_get_state) 
           [arg vz, arg vdx, arg vdy, arg vdz, arg phi, arg theta, arg psi]

main = reify spec >>= 

  compileWith (CSettings "foo" ".") "foo"
