from sot_talos_balance.filter_differentiator import FilterDifferentiator

def create_chebi1_checby2_series_filter(name, dt, size):
   #b1,a1=cheby2(2, 20,0.05);
   #b2,a2 = cheby1(4,0.05,0.08);
   #(b,a) = filter_series(b1,a1,b2,a2);
   lp_filter = FilterDifferentiator(name);
   
   lp_filter.init(dt, size, 
                  (2.16439898e-05, 4.43473520e-05, -1.74065002e-05, -8.02197247e-05,  -1.74065002e-05,   4.43473520e-05, 2.16439898e-05),
                  (1.,-5.32595322,11.89749109,-14.26803139, 9.68705647,  -3.52968633,   0.53914042))
   return lp_filter;

def create_butter_lp_filter_Wn_04_N_2(name, dt, size):
    # (b,a) =  butter(N=2, Wn=0.04)
    lp_filter = FilterDifferentiator(name)

    lp_filter.init(dt, size,
                   (0.0036216815,   0.007243363, 0.0036216815),
                   (          1., -1.8226949252, 0.8371816513))

    return lp_filter

