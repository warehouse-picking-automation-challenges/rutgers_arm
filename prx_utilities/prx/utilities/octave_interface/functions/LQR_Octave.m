function [costMatrix,gainMatrix] = LQR_Octave (A,B,Q,R)
	[S,K,e] = lqr(A,B,Q,R);
	costMatrix = K;
	gainMatrix = S;
endfunction
