/**
 * File: Demo.cpp
 * Date: November 2011
 * Author: Dorian Galvez-Lopez
 * Description: demo application of DBoW2
 * License: see the LICENSE.txt file
 */

#include <iostream>
#include <vector>

// DBoW2
#include "DBoW2.h" // defines OrbVocabulary and OrbDatabase

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>

#include <cstdlib>

using namespace DBoW2;
using namespace std;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void loadFeatures(vector<vector<cv::Mat > > &features, string image_dir, string image_list_file);
void changeStructure(const cv::Mat &plain, vector<cv::Mat> &out);
void testVocCreation(const vector<vector<cv::Mat > > &features);
void testDatabase(const vector<vector<cv::Mat > > &features);


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void wait()
{
  cout << endl << "Press enter to continue" << endl;
  getchar();
}

// ----------------------------------------------------------------------------

int main(int argc, char** argv)
{
  string image_dir = argv[1], image_list_file = argv[2];

  // cout << (unsigned long int)time(NULL) << "\n";

  vector<vector<cv::Mat > > features;
  loadFeatures(features, image_dir, image_list_file);
  // cout << "features size: " << features.size() << "\n";

  testVocCreation(features);

  testDatabase(features);

  return 0;
}

// ----------------------------------------------------------------------------

void loadFeatures(vector<vector<cv::Mat > > &features, string image_dir, string image_list_file)
{
  cv::theRNG().state = (unsigned long int)time(NULL);

  features.clear();

  cv::Ptr<cv::ORB> orb = cv::ORB::create();

  cout << "Extracting ORB features..." << endl;

  string image_file_name;

  ifstream f(image_list_file);

  while(f >> image_file_name)
  {
    cv::Mat image_left = cv::imread(image_dir + "/images_left/" + image_file_name, 0);
    cv::Mat image_right = cv::imread(image_dir + "/images_right/" + image_file_name, 0);
    cv::Mat mask;
    vector<cv::KeyPoint> keypoints;

    // Use random feature vector as descriptor
    // cv::Mat descriptors(1200, 32, CV_16U);         // ORB size random feature vector
    cv::Mat descriptors(1200, 48, CV_16U);            // LoFTR size random feature vector

    cv::randu(descriptors, cv::Scalar(0), cv::Scalar(300));

    // Use ORB descriptor
    // cv::Mat descriptors;
    // orb->detectAndCompute(image_left, mask, keypoints, descriptors);

    features.push_back(vector<cv::Mat >());
    changeStructure(descriptors, features.back());
  }

  // for(int i = 0; i < NIMAGES; ++i)
  // {
  //   stringstream ss;
  //   ss << "demo/images/image" << i << ".png";

  //   cv::Mat image = cv::imread(ss.str(), 0);
  //   cv::Mat mask;
  //   vector<cv::KeyPoint> keypoints;
  //   cv::Mat descriptors;

  //   orb->detectAndCompute(image, mask, keypoints, descriptors);

  //   features.push_back(vector<cv::Mat >());
  //   changeStructure(descriptors, features.back());
  // }
}

// ----------------------------------------------------------------------------

void changeStructure(const cv::Mat &plain, vector<cv::Mat> &out)
{
  out.resize(plain.rows);

  for(int i = 0; i < plain.rows; ++i)
  {
    out[i] = plain.row(i);
  }
}

// ----------------------------------------------------------------------------

void testVocCreation(const vector<vector<cv::Mat > > &features)
{
  int NIMAGES = features.size();

  // branching factor and depth levels 
  const int k = 11;
  const int L = 3;
  const WeightingType weight = TF_IDF;
  const ScoringType scoring = L1_NORM;

  OrbVocabulary voc(k, L, weight, scoring);

  cout << "Creating a small " << k << "^" << L << " vocabulary..." << endl;
  voc.create(features);
  cout << "... done!" << endl;

  cout << "Vocabulary information: " << endl
  << voc << endl << endl;

  // lets do something with this vocabulary
  // cout << "Matching images against themselves (0 low, 1 high): " << endl;
  // BowVector v1, v2;
  // for(int i = 0; i < NIMAGES; i++)
  // {
  //   voc.transform(features[i], v1);
  //   for(int j = 0; j < NIMAGES; j++)
  //   {
  //     voc.transform(features[j], v2);
      
  //     double score = voc.score(v1, v2);
  //     cout << "Image " << i << " vs Image " << j << ": " << score << endl;
  //   }
  // }

  // save the vocabulary to disk
  cout << endl << "Saving vocabulary..." << endl;
  voc.save("small_voc.yml.gz");
  cout << "Done" << endl;
}

// ----------------------------------------------------------------------------

void testDatabase(const vector<vector<cv::Mat > > &features)
{
  int NIMAGES = features.size();

  cout << "Creating a small database..." << endl;

  // load the vocabulary from disk
  OrbVocabulary voc("small_voc.yml.gz");
  
  OrbDatabase db(voc, false, 0); // false = do not use direct index
  // (so ignore the last param)
  // The direct index is useful if we want to retrieve the features that 
  // belong to some vocabulary node.
  // db creates a copy of the vocabulary, we may get rid of "voc" now

  // add images to the database
  for(int i = 0; i < NIMAGES; i++)
  {
    db.add(features[i]);
  }

  cout << "... done!" << endl;

  cout << "Database information: " << endl << db << endl;

  // and query the database
  cout << "Querying the database: " << endl;

  QueryResults ret;
  for(int i = 0; i < NIMAGES; i++)
  {
    db.query(features[i], ret, 4);

    // ret[0] is always the same image in this case, because we added it to the 
    // database. ret[1] is the second best match.

    cout << "Searching for Image " << i << ". " << ret << endl;
  }

  cout << endl;

  // we can save the database. The created file includes the vocabulary
  // and the entries added
  cout << "Saving database..." << endl;
  db.save("small_db.yml.gz");
  cout << "... done!" << endl;
  
  // once saved, we can load it again  
  cout << "Retrieving database once again..." << endl;
  OrbDatabase db2("small_db.yml.gz");
  cout << "... done! This is: " << endl << db2 << endl;
}

// ----------------------------------------------------------------------------