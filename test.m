a = [];
a(1) = 0;
a(2) = 1;

b = [];
b(1) = 1;
b(2) = 2;

for i=3:30
    a(i) = (i-1)*(a(i-1) + a(i-2));
    b(i) = i * b(i-1);
end
c = [];

for i=1:29
    c(i) = a(30-i)/(b(i)*b(30-i)); 
end


c(30) = 0;

e = 0;
for i=1:30
    e = i*c(i) + e;
end