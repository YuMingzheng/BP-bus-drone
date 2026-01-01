\ENCODING=ISO-8859-1
\Problem name: ilog.cplex

Minimize
 obj: 97.7838506334473 P2 + 153.143877654401 P3 + 254.135099629714 P4
      + 245.573223898182 P5 + 87.729451273339 P6 + 89.5726204548275 P7
Subject To
 c1: P4  = 1
 c1: P2 + P6 + P7  = 1
 c2: P3  = 1
 c3: P5  = 1
 c4: P4  = 1
 c5: P5  = 1
 c6: P4  = 1
 c7: P2 + P3 + P4 + P5 + P6 + P7 <= 4
Bounds
 0 <= P2 <= 1
 0 <= P3 <= 1
 0 <= P4 <= 1
 0 <= P5 <= 1
 0 <= P6 <= 1
 0 <= P7 <= 1
Generals
 P2  P3  P4  P5  P6  P7 
End
