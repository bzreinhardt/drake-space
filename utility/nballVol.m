function v = nballVol(r,n)

if mod(n,2) == 0
	k = n/2;
	v = pi^k/factorial(k)*r^(n);
else
k = (n-1)/2;	
v = 2*factorial(k).*(4*pi).^k./factorial(n).*r.^(n);

end
	

end
