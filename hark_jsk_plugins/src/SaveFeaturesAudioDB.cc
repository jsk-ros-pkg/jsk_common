/*
 * Copyright 2008 Kyoto University and Honda Motor Co.,Ltd.
 * All rights reserved.
 * HARK was developed by researchers in Okuno Laboratory
 * at the Kyoto University and Honda Research Institute Japan Co.,Ltd.
 */

#include "BufferedNode.h"
#include "Buffer.h"
#include "Vector.h"
#include "Map.h"
#include "Source.h"
#include <sstream>
#include <iomanip>

#include <boost/shared_ptr.hpp>

using namespace std;
using namespace FD;
using namespace boost;

class SaveFeaturesAudioDB;

DECLARE_NODE(SaveFeaturesAudioDB)
/*Node
 *
 * @name SaveFeaturesAudioDB
 * @category JSK
 * @description Save a sequence of feature vectors as a binary file per utterace. The file format depends on fftExtract.
 *
 * @input_name FEATURES
 * @input_type Map<int,ObjectRef>
 * @input_description Feature vectors. The key is source ID, the value is a feature vector (Vector<double>).
 *
 * @input_name SOURCES
 * @input_type Vector<ObjectRef>
 * @input_description Source locations with ID. Each element of the vector is a source location with ID specified by "Source". Feature vectors are saved for each source ID. This parameter is optional.
 *
 * @output_name OUTPUT
 * @output_type Map<int,ObjectRef>
 * @output_description The same as FEATURE.
 * @parameter_name BASENAME
 *
 * @parameter_type string
 * @parameter_value sep_
 * @parameter_description Base name for save-file.
 *
 * @parameter_name SUFFIX
 * @parameter_type string
 * @parameter_value .spec
 * @parameter_description suffix name for save-file.
 *
 * @parameter_name HOP
 * @parameter_type int
 * @parameter_value 1000
 * @parameter_description (NOT IMPREMENTED YET) save feature interval (frames)
 *
 * @parameter_name WITH_ID
 * @parameter_type bool
 * @parameter_value true
 * @parameter_list true:false
 * @parameter_description add id to the filename's suffix
END*/

class SaveFeaturesAudioDB : public BufferedNode {
  int featuresID;
  int outputID;
  int sourcesID;
  string baseName;
  string suffixName;
  int hop;
  bool withid;
  map<int, shared_ptr<ofstream> > streams;
    
public:
  SaveFeaturesAudioDB(string nodeName, ParameterSet params)/*{{{*/
    : BufferedNode(nodeName, params), sourcesID(-1)
  {
    featuresID = addInput("FEATURES");
    outputID = addOutput("OUTPUT");
    baseName = object_cast<String> (parameters.get("BASENAME"));
    suffixName = object_cast<String> (parameters.get("SUFFIX"));
    hop = dereference_cast<int> (parameters.get("HOP"));
    withid = dereference_cast<bool> (parameters.get("WITH_ID"));

    inOrder = true;
  }/*}}}*/
    
  ~SaveFeaturesAudioDB() {
  }
    
  void reset()/*{{{*/
  {
    streams.clear();
  }/*}}}*/

  virtual int translateInput(string inputName)/*{{{*/
  {
    for (unsigned int i = 0; i < inputs.size(); i++) {
      if (inputs[i].name == inputName) {
        return i;
      }
    }
    if (inputName == "SOURCES") {
      return sourcesID = addInput(inputName);
    }
    else {
      throw new NodeException(this, inputName + " is not supported.", __FILE__, __LINE__);
    }
  }/*}}}*/
  
  void calculate(int output_id, int count, Buffer &out)
  {
    RCPtr<Map<int, ObjectRef> > input = getInput(featuresID, count);
    RCPtr<Vector<ObjectRef> > sources;
    if (sourcesID != -1) {
      sources = getInput(sourcesID, count);
    }

    if (sources.isNil()) {
      Map<int, ObjectRef>::const_iterator it;
      for (it = input->begin(); it != input->end(); ++it) {
        const Vector<float>& feature = object_cast<Vector<float> >(it->second);
        if (streams.find(it->first) == streams.end()) {
          ostringstream filename;
          if(withid){
              filename << baseName << it->first << suffixName;
          }else{
              filename << baseName << suffixName;
          }
          streams[it->first].reset(new ofstream(filename.str().c_str(),ofstream::binary));
          int i = feature.size();
          (streams[it->first])->write((const char*)(&i),sizeof(int));
        }
        
        ofstream& stream = *streams[it->first];
        
        Vector<float>::const_iterator tmp_itr;
        for(tmp_itr = feature.begin();tmp_itr != feature.end();tmp_itr++){
            double d = (double)*tmp_itr;
            stream.write((const char*)(&d), sizeof(double));
        }
      }
    }
    else {
      for (int i = 0; i < sources->size(); i++) {
        RCPtr<Source> src = (*sources)[i];
        if (streams.find(src->id) == streams.end()) {
          ostringstream filename;
          if(withid){
              filename << baseName << src->id << suffixName;
          }else{
              filename << baseName << suffixName;
          }
          streams[src->id].reset(new ofstream(filename.str().c_str(),ofstream::binary));
          
          Map<int, ObjectRef>::iterator it_tmp = input->find(src->id);
          const Vector<float>& feature_tmp = object_cast<Vector<float> >(it_tmp->second);
          int i = feature_tmp.size();
          (streams[src->id])->write((const char*)(&i),sizeof(int));
        }
        ofstream& stream = *streams[src->id];

        Map<int, ObjectRef>::iterator it = input->find(src->id);
        if (it != input->end()) {
          const Vector<float>& feature = object_cast<Vector<float> >(it->second);
          //stream.write((const char*)&feature[0], sizeof(double) * feature.size());
          Vector<float>::const_iterator tmp_itr;
          for(tmp_itr = feature.begin();tmp_itr != feature.end();tmp_itr++){
              double d = (double)*tmp_itr;
              stream.write((const char*)(&d), sizeof(double));
          }
        }
      }
    }
    out[count] = input;
  }
};
