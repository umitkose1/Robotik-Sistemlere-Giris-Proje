# Robotik-Sistemlere-Giris-Proje

## GMapping ile Harita Olusturma

Bu baslik altinda gmapping kullanarak onceden kendinizin kaydetmis oldugu bir rosbag dosyasi veya hazir olarak linkten indirebileceginiz bir rosbag dosyasi uzerinden harita cikartacagiz.

gmapping Kurulum::

Indir = https://github.com/ros-perception/slam_gmapping

image::https://github.com/mayuce/pathPlanning/blob/master/Images-for-ReadMe/gmapping/10.png?raw=true[]

Adresinden indirdigimiz arsivdeki dosyalari masaustune cikartin.

```bourne
rosws init <cikarttiginiz arsivin adresi> </opt/ros/indigo/share == yani $ROS_PACKAGE_PATH ' iniz. Turtlebot imaji icin esitligin solundaki>
```

image::https://github.com/mayuce/pathPlanning/blob/master/Images-for-ReadMe/gmapping/11.png?raw=true[]

artik gmapping kodunuz kullanima hazir. Not: Bende onceden yukledigim icin " Error " uyarisinin aldim.


Adimlar::

* Asagidaki linten dosyayi indirin veya rosbag kullanarak kendi kaydinizi alin.
 
Indir = http://wiki.ros.org/slam_gmapping/Tutorials/MappingFromLoggedData#download

* roscore'i calistirin

image::https://github.com/mayuce/pathPlanning/blob/master/Images-for-ReadMe/gmapping/1.png?raw=true[]

```bourne
 rosparam set use_sim_time true
```
 komutunu calistirin

image::https://github.com/mayuce/pathPlanning/blob/master/Images-for-ReadMe/gmapping/2.png?raw=true[]

```bourne
rosrun gmapping slam_gmapping scan:=base_scan
```

komut ile gmapping algoritmasini lazer tarayici uzerinden calistirmak icin baslatin. Eger odometri verisini de
kullanacaksaniz asagidaki komut ile calistirin.

```bourne
rosrun gmapping slam_gmapping scan:=base_scan _odom_frame:=odom_combined
```

bu komut ile odometri verisi de izlenecek. Indirdiginiz dosya icin sadece base_scan olanini calistirmaniz yeterli olacaktir.

image::https://github.com/mayuce/pathPlanning/blob/master/Images-for-ReadMe/gmapping/3.png?raw=true[]

```bourne
rosbag play --clock <indirdiginiz dosyanin yeri veya olusturdugunuz rosbag'in yeri>
```

bu komut ile bag'daki veriler sanki su an robot uzerinden aliniyormus gibi topiclere iletilecektir.

image::https://github.com/mayuce/pathPlanning/blob/master/Images-for-ReadMe/gmapping/5.png?raw=true[]

kodu bitince asagidaki gibi bir ekran karsilayacak sizi. 

image::https://github.com/mayuce/pathPlanning/blob/master/Images-for-ReadMe/gmapping/6.png?raw=true[]

```bourne
rosrun map_server map_saver -f <kaydetmek istediginiz yer>
```

son olarak bu komut ile dosyamizi kaydediyoruz ve haritamiz olusuyor.

image::https://github.com/mayuce/pathPlanning/blob/master/Images-for-ReadMe/gmapping/8.png?raw=true[]

olusan harita:

image::https://github.com/mayuce/pathPlanning/blob/master/Images-for-ReadMe/gmapping/9.png?raw=true[]

## PDF Aciklamasi

PDF: http://www.informatik.uni-freiburg.de/~stachnis/pdf/grisetti07tro.pdf

Algoritmanın Açıklaması::
slam_gmapping gmapping SLAM kütüphanesi etrafında bir sarıcıdır. Lazer ve odometri verisini
kullanarak haritayı hesaplar. Bu harita bir dosyaya yazılır.

Kullanılan Değişkenler(rostopic'ler)::

Abone Olduğu (isim/tip):
"scan"/<a href="../../sensor_msgs/html/classstd__msgs_1_1LaserScan.html">sensor_msgs/LaserScan</a> : lazer uzaklık tarayıcısının verisi
"/tf": robotun odometrisi


Yayınladığı (isim/tip):
- @b "/tf"/tf/tfMessage: haritayla ilişkili konum



Kod::

```java
int
main(int argc, char** argv)
{

  ros::init(argc, argv, "slam_gmapping"); //Burada ROS topic'leri ve gMapping parametrelerinin ataması yapılıyor.
  SlamGMapping gn;
  gn.startLiveSlam(); // gMapping işlemi başlatılıyor bu fonksiyon gerçek zamanlı gMapping yapabiliyor. //startReplay(arg,arg) isimli fonksiyon ile direkt rosbag dosyasının haritasını'da elde edebiliriz.
  
  ros::spin(); // kullanıcı haritayı kaydedebilsin diye program loopda tutuluyor.
  return(0);
}
```




```java
double
SlamGMapping::computePoseEntropy()
{
  double weight_total=0.0;
  for(std::vector<GMapping::GridSlamProcessor::Particle>::const_iterator it = gsp_->getParticles().begin();
      it != gsp_->getParticles().end();
      ++it)
  {
    weight_total += it->weight;
  }
  double entropy = 0.0;
  for(std::vector<GMapping::GridSlamProcessor::Particle>::const_iterator it = gsp_->getParticles().begin();
      it != gsp_->getParticles().end();
      ++it)
  {
    if(it->weight/weight_total > 0.0)
      entropy += it->weight/weight_total * log(it->weight/weight_total);
  }
  return -entropy;
}
```

buradaki kod ile agirlik hesaplamasi yapiliyor.

```java
void
SlamGMapping::updateMap(const sensor_msgs::LaserScan& scan)
{
  ROS_DEBUG("Update map");
  boost::mutex::scoped_lock map_lock (map_mutex_);
  GMapping::ScanMatcher matcher;

  matcher.setLaserParameters(scan.ranges.size(), &(laser_angles_[0]),
                             gsp_laser_->getPose());

  matcher.setlaserMaxRange(maxRange_);
  matcher.setusableRange(maxUrange_);
  matcher.setgenerateMap(true);

  GMapping::GridSlamProcessor::Particle best =
          gsp_->getParticles()[gsp_->getBestParticleIndex()];
  std_msgs::Float64 entropy;
  entropy.data = computePoseEntropy();
  if(entropy.data > 0.0)
    entropy_publisher_.publish(entropy);

  if(!got_map_) {
    map_.map.info.resolution = delta_;
    map_.map.info.origin.position.x = 0.0;
    map_.map.info.origin.position.y = 0.0;
    map_.map.info.origin.position.z = 0.0;
    map_.map.info.origin.orientation.x = 0.0;
    map_.map.info.origin.orientation.y = 0.0;
    map_.map.info.origin.orientation.z = 0.0;
    map_.map.info.origin.orientation.w = 1.0;
  } 

  GMapping::Point center;
  center.x=(xmin_ + xmax_) / 2.0;
  center.y=(ymin_ + ymax_) / 2.0;

  GMapping::ScanMatcherMap smap(center, xmin_, ymin_, xmax_, ymax_, 
                                delta_);

  ROS_DEBUG("Trajectory tree:");
  for(GMapping::GridSlamProcessor::TNode* n = best.node;
      n;
      n = n->parent)
  {
    ROS_DEBUG("  %.3f %.3f %.3f",
              n->pose.x,
              n->pose.y,
              n->pose.theta);
    if(!n->reading)
    {
      ROS_DEBUG("Reading is NULL");
      continue;
    }
    matcher.invalidateActiveArea();
    matcher.computeActiveArea(smap, n->pose, &((*n->reading)[0]));
    matcher.registerScan(smap, n->pose, &((*n->reading)[0]));
  }

  // the map may have expanded, so resize ros message as well
  if(map_.map.info.width != (unsigned int) smap.getMapSizeX() || map_.map.info.height != (unsigned int) smap.getMapSizeY()) {

    // NOTE: The results of ScanMatcherMap::getSize() are different from the parameters given to the constructor
    //       so we must obtain the bounding box in a different way
    GMapping::Point wmin = smap.map2world(GMapping::IntPoint(0, 0));
    GMapping::Point wmax = smap.map2world(GMapping::IntPoint(smap.getMapSizeX(), smap.getMapSizeY()));
    xmin_ = wmin.x; ymin_ = wmin.y;
    xmax_ = wmax.x; ymax_ = wmax.y;
    
    ROS_DEBUG("map size is now %dx%d pixels (%f,%f)-(%f, %f)", smap.getMapSizeX(), smap.getMapSizeY(),
              xmin_, ymin_, xmax_, ymax_);

    map_.map.info.width = smap.getMapSizeX();
    map_.map.info.height = smap.getMapSizeY();
    map_.map.info.origin.position.x = xmin_;
    map_.map.info.origin.position.y = ymin_;
    map_.map.data.resize(map_.map.info.width * map_.map.info.height);

    ROS_DEBUG("map origin: (%f, %f)", map_.map.info.origin.position.x, map_.map.info.origin.position.y);
  }

  for(int x=0; x < smap.getMapSizeX(); x++)
  {
    for(int y=0; y < smap.getMapSizeY(); y++)
    {
      /// @todo Sort out the unknown vs. free vs. obstacle thresholding
      GMapping::IntPoint p(x, y);
      double occ=smap.cell(p);
      assert(occ <= 1.0);
      if(occ < 0)
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = -1;
      else if(occ > occ_thresh_)
      {
        //map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = (int)round(occ*100.0);
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 100;
      }
      else
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 0;
    }
  }
  got_map_ = true;

  //make sure to set the header information on the map
  map_.map.header.stamp = ros::Time::now();
  map_.map.header.frame_id = tf_.resolve( map_frame_ );

  sst_.publish(map_.map);
  sstm_.publish(map_.map.info);
}
```

kodun bu parcasinda ise harita guncelleme adimi islemi yapiliyor.


