function cost = power_cost(ch, k, K, x, params, p_reference)

  e = x.positionNav - p_reference(:,k);
  cost = e'*e;

  mu = params.mu;
  cost = params.w_referenceTracking * (1-mu) * cost / x.time;

  ch.add(cost);

  if k==K
    integratedWork = x.integratedWork;
    endTime = params.endTime;
    ch.add( -params.w_integratedWork*mu*integratedWork/endTime);
  end

end
