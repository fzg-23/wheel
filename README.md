# WBR Control
**Wheeled Bipedal Robot POW** project for Capstone Design graduation thesis conducted. This project is inspired by [Ascento](https://www.youtube.com/@AscentoRobotics). @SeungbinOh, Jungbin Park, @Woodaengtang are contributors of this project. 

## Hardware

<img src="/assets/hw_schemetic.png" width="700" heigh="370">

- [DCDC step down converter](https://ko.aliexpress.com/item/1005006295829518.html?spm=a2g0o.productlist.main.1.16534b99MvfQUP&algo_pvid=870918c7-344b-40ee-95af-d3b5d3e207b9&algo_exp_id=870918c7-344b-40ee-95af-d3b5d3e207b9-0&pdp_npi=4%40dis%21KRW%217122%211359%21%21%2136.60%216.98%21%402141115b17282419275056704e9eae%2112000036649730449%21sea%21KR%216062351685%21ABX&curPageLogUid=S7a0WnK8hPSP&utparam-url=scene%3Asearch%7Cquery_from%3A)
- [DC-DC buck converter](https://ko.aliexpress.com/item/1005001711248152.html?src=google&src=google&albch=shopping&acnt=298-731-3000&isdl=y&slnk=&plac=&mtctp=&albbt=Google_7_shopping&aff_platform=google&aff_short_key=UneMJZVf&gclsrc=aw.ds&&albagn=888888&&ds_e_adid=&ds_e_matchtype=&ds_e_device=c&ds_e_network=x&ds_e_product_group_id=&ds_e_product_id=ko1005001711248152&ds_e_product_merchant_id=107637876&ds_e_product_country=KR&ds_e_product_language=ko&ds_e_product_channel=online&ds_e_product_store_id=&ds_url_v=2&albcp=21523018537&albag=&isSmbAutoCall=false&needSmbHouyi=false&gad_source=1&gclid=Cj0KCQjw6oi4BhD1ARIsAL6pox1GZ2XxRJhE5b5-iDoPZDh7TXWR3Q2C-OobytdxC6nKLmC6wSKew3YaAtAnEALw_wcB)
- [ESP32 N16R8](https://ko.aliexpress.com/item/1005006002965361.html?spm=a2g0o.productlist.main.3.7159ec79YMMyNV&algo_pvid=8703ea9a-c4a0-45b9-9042-893e83eafef3&algo_exp_id=8703ea9a-c4a0-45b9-9042-893e83eafef3-2&pdp_npi=4%40dis%21KRW%217757%211359%21%21%215.65%210.99%21%4021015b2417282315146826646e0261%2112000035266063777%21sea%21KR%216062351685%21ABX&curPageLogUid=1dmMOIvf8EqB&utparam-url=scene%3Asearch%7Cquery_from%3A)
- [Hip servo motor](https://ko.aliexpress.com/item/1005006086271452.html?spm=a2g0o.productlist.main.1.23a84e73qVf3u4&algo_pvid=4f871638-fd94-497a-a7ee-8f393fc3189e&algo_exp_id=4f871638-fd94-497a-a7ee-8f393fc3189e-0&pdp_npi=4%40dis%21KRW%21110377%2126671%21%21%21567.27%21137.07%21%40212e520f17282087547377488e809f%2112000035664779393%21sea%21KR%216059889447%21ABX&curPageLogUid=RGJV35dx32iU&utparam-url=scene%3Asearch%7Cquery_from%3A)
- [Wheel motor](https://ko.aliexpress.com/item/1005006136735214.html?pdp_npi=4%40dis!KRW!%E2%82%A9%20176%2C600!%E2%82%A9%20176%2C600!!!128.63!128.63!%40213ba0c117282090894563426e1db9!12000035922232220!sh!KR!0!X&spm=a2g0o.store_pc_allItems_or_groupList.new_all_items_2007602073851.1005006136735214&gatewayAdapt=glo2kor)
- [RC Receiver](https://ko.aliexpress.com/item/1005006821905179.html?spm=a2g0o.productlist.main.1.6ff22b8cEJjWYh&algo_pvid=1b1d5714-ebed-4917-9651-eebc49c7e525&algo_exp_id=1b1d5714-ebed-4917-9651-eebc49c7e525-0&pdp_npi=4%40dis%21KRW%2149611%214744%21%21%21254.97%2124.38%21%402101584917282272920287406e04f0%2112000038412056083%21sea%21KR%210%21ABX&curPageLogUid=SHq5mZON0Lqz&utparam-url=scene%3Asearch%7Cquery_from%3A)
- [MPU6050](https://ko.aliexpress.com/item/1005006579363624.html?spm=a2g0o.order_detail.order_detail_item.3.57845ccdSI5Mk7&gatewayAdapt=glo2kor)
- [TTL to RS485 converter](https://www.coupang.com/vp/products/104771330?itemId=317638085&vendorItemId=3992447841&q=MAX485&itemsCount=36&searchId=46c295beff234d5ca881c19897701951&rank=1&searchRank=1&isAddedCart=)
- [Level shifter](https://www.coupang.com/vp/products/257783456?itemId=808722571&vendorItemId=5056810708&q=%EB%A0%88%EB%B2%A8+%EC%8B%9C%ED%94%84%ED%84%B0&itemsCount=36&searchId=0ac0aa5ec7e2476fa8a5267abd0d458e&rank=1&searchRank=1&isAddedCart=)

## Library dependencies
- **ESP32 Board**
    - [arduino-esp32](https://github.com/espressif/arduino-esp32): 4D Systems gen4-ESP32 16MB Modules (ESP32-S3R8n16)
- [Eigen](https://github.com/hideakitai/ArduinoEigen)
- [Bolder Flight Systems SUBS](https://github.com/bolderflight/sbus)
- [ESP32Servo](https://madhephaestus.github.io/ESP32Servo/annotated.html)


