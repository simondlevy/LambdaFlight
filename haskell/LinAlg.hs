module Main where

import Linear.Matrix

type Matrix = [[Float]]

msq :: Matrix -> Matrix
msq a = a !*! a


main = do

    let m = [ [ 0,1,2,3,4 ],
              [ 1,2,3,4,0 ],
              [ 2,3,4,0,1 ],
              [ 3,4,0,1,2 ],
              [ 4,0,1,2,3 ] ] :: Matrix

    print $ msq m

