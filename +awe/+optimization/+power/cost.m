function c = cost(ch, k, K, x, params, p_reference)

  e = x.positionNav - p_reference(:,k);
  c = e'*e;

  mu = params.mu;
  c = params.w_referenceTracking * (1-mu) * c / x.time;

  ch.add(c);

  if k==K
    integratedWork = x.integratedWork;
    endTime = params.endTime;
    ch.add( -params.w_integratedWork*mu*integratedWork/endTime);
  end

end
