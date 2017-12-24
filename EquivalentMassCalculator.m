function [mass_equivalent] = EquivalentMassCalculator(inertia, effectiveRadius)
%EquivalentMassCalculator - Calculates the "Equivalent Mass" of a rotating
%component by using m + I/r^2

mass_equivalent = (inertia/(effectiveRadius^2));

end