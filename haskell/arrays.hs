import Data.Array

wavefront       :: Array (Int,Int) Float

wavefront =  a  where
  a = array ((1,1),(3,3)) [((1,1),1.0),((1,2),1.0),((1,3),1.0),((2,1),1.0),((2,2),3.0),((2,3),5.0),((3,1),1.0),((3,2),5.0),((3,3),13.0)]

main = do 

  let arr =  wavefront

  print arr
