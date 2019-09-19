#! /usr/bin/awk -f

BEGIN {
  printf "unset key\n"
  printf "set style line 100 lt 1 lc rgb \"black\" lw 2\n"
  printf "set style line 101 lt 1 lc rgb \"gray\" lw 1\n"
  printf "set grid xtics, mxtics lt -1 lw 0.5, lt 0 lw 1\n"
  printf "set grid ytics, mytics lt -1 lw 0.5, lt 0 lw 1\n"
  printf "set  xtics 1.0\n"
  printf "set mxtics 5\n"
  printf "set  ytics 0.1\n"
  printf "set mytics 5\n"
  printf "set xlabel 't[sec]'\n"
  printf "set ylabel 'men x[m]'\n"
  printf "plot [0:10][-0.2:0.8] 0\n"
}

NR==1 {
  offset = $1;
  printf "#offset = %d\n", offset
}

{
  printf "set label %d point pt 6 at %f,%f\n",
         NR,
         ($1-offset)/1000.0,
         $2
  printf "replot [%f:%f] %e + %e*(x-(%e)) + %e*((x-(%e))**2)\n",
         ($1-offset-400)/1000.0,
         ($1-offset+200)/1000.0,
         $3,
         $4 * 1000,
         ($6-offset)/1000.0,
         $5 * 1000 * 1000,
         ($6-offset)/1000.0
}
