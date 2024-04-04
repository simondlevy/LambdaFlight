module Main where

import Linear.Matrix

m :: [[Float]]
m = [ [ 0,1,2,3,4 ],
      [ 1,2,3,4,0 ],
      [ 2,3,4,0,1 ],
      [ 3,4,0,1,2 ],
      [ 4,0,1,2,3 ] ]

msq :: [[Float]]
msq = m !*! m


main = do

    print msq

