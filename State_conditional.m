%stateConditional
%
%Determines whether the pod should switch to the deceleration state based
%on position and estimated braking distance.

 
function [state] = State_conditional(decelerationDistance,s,trialDistance)
if s + decelerationDistance > trialDistance %&& s + decelerationDistance <= trialDistance + 1
    state = 'dec';
else
    state = 'acc';
end
end


