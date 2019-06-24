function c = point_cost(ch, k, K, x, params, ref_p)

  e = x.p - ref_p(:,k);
  c = e'*e;

  mu = params.mu;
  c = conf.w_referenceTracking * (1-mu) * c / x.time;

  ch.add(c);

  if k==K
    integratedWork = x.integratedWork;
    endTime = x.time;
    ch.add( -conf.w_integratedWork * mu * integratedWork / endTime);
  end

end
