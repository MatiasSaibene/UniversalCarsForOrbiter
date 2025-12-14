#include "main.hpp"

void UCFO::MakeAnnotation_Format(){

    msg1_annotation = oapiCreateAnnotation(true, 0.8, _V(0, 1, 0));
    oapiAnnotationSetPos(msg1_annotation, 0.01, 0.42, 0.5, 1);

    msg2_annotation = oapiCreateAnnotation(true, 0.8, _V(0, 1, 0));
    oapiAnnotationSetPos(msg2_annotation, 0.01, 0.45, 0.5, 1);

    msg3_annotation = oapiCreateAnnotation(true, 0.8, _V(1, 0, 0));
    oapiAnnotationSetPos(msg3_annotation, 0.01, 0.50, 0.5, 1);
    
    msg4_annotation = oapiCreateAnnotation(true, 0.8, _V(1, 0, 0));
    oapiAnnotationSetPos(msg4_annotation, 0.01, 0.52, 0.5, 1);

    msg5_annotation = oapiCreateAnnotation(true, 0.8, _V(1, 0, 0));
    oapiAnnotationSetPos(msg5_annotation, 0.01, 0.54, 0.5, 1);

    msg6_annotation = oapiCreateAnnotation(true, 0.8, _V(1, 0, 0));
    oapiAnnotationSetPos(msg6_annotation, 0.01, 0.58, 0.5, 1);

    msg7_annotation = oapiCreateAnnotation(true, 0.8, _V(1, 0, 0));
    oapiAnnotationSetPos(msg7_annotation, 0.01, 0.72, 0.5, 1);

    msg8_annotation = oapiCreateAnnotation(true, 0.8, _V(1, 0, 0));
    oapiAnnotationSetPos(msg8_annotation, 0.01, 0.76, 0.5, 1);

    msg9_annotation = oapiCreateAnnotation(true, 0.8, _V(1, 0, 0));
    oapiAnnotationSetPos(msg9_annotation, 0.01, 0.80, 0.5, 1);

    msg10_annotation = oapiCreateAnnotation(true, 0.6, _V(1, 0, 0));
    oapiAnnotationSetPos(msg10_annotation, 0.01, 0.84, 0.5, 1);

    msg11_annotation = oapiCreateAnnotation(true, 0.6, _V(1, 0, 0));
    oapiAnnotationSetPos(msg11_annotation, 0.01, 0.88, 0.5, 1);

    msg12_annotation = oapiCreateAnnotation(true, 0.6, _V(1, 0, 0));
    oapiAnnotationSetPos(msg12_annotation, 0.01, 0.92, 0.5, 1);

}