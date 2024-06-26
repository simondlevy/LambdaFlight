module LinAlg where

import Linear.Matrix hiding(transpose)
import Data.List

type Vector = [Float]

type Matrix = [Vector]

printvec' :: Vector -> Int -> IO()
printvec' _ 0 = do
  putStrLn "\n"  
printvec' x cols = do
    putStr . show $ head x
    putStr " "
    printvec' (tail x) (cols - 1)


printvec :: Vector -> IO()
printvec x = do
    printvec' x (length x)

printmat' :: Matrix -> Int -> IO()
printmat' _ 0 = do
  putStr ""  
printmat' a rows = do
    printvec $ head a
    printmat' (tail a) (rows - 1)

 
printmat :: Matrix -> IO()
printmat a = do
    printmat' a (length a)

dot :: Vector -> Vector -> Float
dot (x:xs) (y:ys) = x * y + (dot xs ys)
dot [] [] = 0

outer :: Vector -> Vector -> Matrix
outer x y = map (!!0) [[y] !!* k | k <- x]

idmat = [[1, 0, 0],
         [0, 1, 0],
         [0, 0, 1]]

main = do

    let a = [ [ 2,   3,  5],
              [ 7,  11, 13],
              [ 17, 19, 23 ] ]

    let x = [29, 31, 37]

    let v = [0, 2, 4]
    let w = [1, 3, 5]

    -- print $ dot x x

    -- printmat $ map (\x -> x!!0) [[v] !!* k | k <- w]
    printmat $ (outer v w) !-! idmat
  
    -- printvec $ a !* x
    -- printmat $ transpose a
