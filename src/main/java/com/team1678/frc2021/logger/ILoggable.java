package com.team1678.frc2021.logger;
import java.util.ArrayList;

/*  
    See:
         LogStorage for functionality
         TestLoggable for implementation
*/
 
 public interface ILoggable {
     ArrayList<ArrayList<Double>> getItems();
     ArrayList<String> getItemNames();
 }
