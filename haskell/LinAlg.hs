module Main where

import Linear.Matrix

type Matrix = [[Float]]

msq :: Matrix -> Matrix
msq a = a !*! a

printmat :: Matrix -> IO()
printmat a = do
    print $ a!!0
    print $ a!!1
    print $ a!!2
    print $ a!!3
    print $ a!!4

main = do

    let a = [ [ 0,1,2,3,4 ],
              [ 1,2,3,4,0 ],
              [ 2,3,4,0,1 ],
              [ 3,4,0,1,2 ],
              [ 4,0,1,2,3 ] ]

    printmat $ msq a
