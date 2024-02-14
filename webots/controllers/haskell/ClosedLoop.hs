module ClosedLoop where

import Language.Copilot
import Copilot.Compile.C99

import Demands
import State
import Utils

type ClosedLoopController = SBool -> State -> Demands -> Demands

