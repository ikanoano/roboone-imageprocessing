#! /usr/bin/gawk -f

BEGIN {
  printf "unset key\n"
  printf "set style line 100 lt 1 lc rgb \"black\" lw 2\n"
  printf "set style line 101 lt 1 lc rgb \"gray\" lw 1\n"
  printf "set grid xtics, mxtics lt -1 lw 0.5, lt 0 lw 1\n"
  printf "set grid ytics, mytics lt -1 lw 0.5, lt 0 lw 1\n"
  printf "set  xtics 1.0\n"
  printf "set mxtics 2\n"
  printf "set  ytics 0.1\n"
  printf "set mytics 2\n"
  printf "set xlabel 't[sec]'\n"
  printf "set ylabel 'men x[m]'\n"
  printf "set xrange [0:1]\n"
  printf "set yrange [-0.2:1.0]\n"
  printf "plot '< echo 0 0'\n"
}

NR==5 {
  offset = $1;
  printf "#offset = %d\n", offset
}

NR>5 {
  printf "set label %d point pt 12 ps 1 at %f,%f\n",
         NR,
         ($1-offset)/1000.0,
         $2
  printf "unset label %d\n",
         NR-270
  if((NR%600)==0) {
    # clear old graph
    printf "plot '< echo 0 0'\n"
  }
  system("")

  if(NR%5) next
  f = sprintf("%e + %e*(x-(%e)) + %e*((x-(%e))**2)",
    $3,
    $4 * 1000,
    ($6-offset)/1000.0,
    $5 * 1000 * 1000,
    ($6-offset)/1000.0)
  # fact part
  printf "replot [%f:%f] %s lt rgb \"gray\" lw 0.5\n",
         ($1-offset-400)/1000.0,
         ($1-offset    )/1000.0,
         f
  # prediction part
  printf "replot [%f:%f] %s lt rgb \"red\"\n",
         ($1-offset+1  )/1000.0,
         ($1-offset+200)/1000.0,
         f
  if(NR%15) next
  printf "set xrange [%f:%f]\n",
         ($1-offset)/1000.0-9,
         ($1-offset)/1000.0+1
}
