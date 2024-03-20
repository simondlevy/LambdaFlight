{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module Quat where

import Language.Copilot
import Copilot.Compile.C99

import Prelude hiding(atan2, (++))
import Utils

invSqrt :: SFloat -> SFloat
invSqrt x = 1 / (sqrt x)

madgwick6DOF :: (SFloat, SFloat, SFloat) -> (SFloat, SFloat, SFloat) -> 
  (SFloat, SFloat, SFloat)

madgwick6DOF (gx, gy, gz) (ax, ay, az) = (phi, theta, psi) where

  b_madgwick = 0.04 :: SFloat

  -- Convert gyroscope degrees/sec to radians/sec
  ggx = gx * 0.0174533
  ggy = gy * 0.0174533
  ggz = gz * 0.0174533

  -- Normalise accelerometer measurement
  recipNorm = invSqrt $ ax * ax + ay * ay + az * az
  aax = ax * recipNorm
  aay = ay * recipNorm
  aaz = az * recipNorm


  -- XXX
  q0 = q0' * gx
  q1 = q1' * gx
  q2 = q2' * gx
  q3 = q3' * gx

  q0' = [1] ++ q0
  q1' = [0] ++ q1
  q2' = [0] ++ q2
  q3' = [0] ++ q3

  phi = q0

  theta = 0

  psi = 0
