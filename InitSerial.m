%------------------------------------------------------------------------
% Product: Serial Port Configuration
%
% Filename: InitSerial.m
%
% Author: Edward Benitez
%
% File Description: This is the implementation of a script to setting 
% recommended serial parameters for communication with Arduino Mega 2560.
% 
%
%
%------------------------------------------------------------------------
% File History
%
% Date   Init  Notes
%------------------------------------------------------------------------
% 27/01/17 ebenitez  Created.
%------------------------------------------------------------------------
%

function device = InitSerial(com)
    port = serial(com);
    port.BaudRate = 115200;
    port.Terminator ='LF';    
    device = port