module Main where

import Linear.Matrix

type Matrix = [[Float]]

msq :: Matrix -> Matrix
msq a = a !*! a

printmat :: Matrix -> IO()
printmat b = do
    print $ b!!0
    print $ b!!1
    print $ b!!2
    print $ b!!3
    print $ b!!4

main = do

    let a = [ [ 0,1,2,3,4 ],
              [ 1,2,3,4,0 ],
              [ 2,3,4,0,1 ],
              [ 3,4,0,1,2 ],
              [ 4,0,1,2,3 ] ]

    let b = msq a 

    printmat b
