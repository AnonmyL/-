// miniprogram/pages/car/car.js
var util = require('../../utils/util.js');
var TIME=util.formatTime(new Date());
const gcoord = require('gcoord');

Page({

  /**
   * 页面的初始数据
   */
  data: {
    currentData:0,
    winWidth: 0,
    winHeight: 0,
    time: TIME,
    showStep:false,
    result_text: ["39.96174659643712","116.35552389252058"],
    deviceW:"",
    deviceH:"",
    car_longitude:"116.35765585552397",//小车经度
    car_latitude:"39.96095349639502",//小车纬度
    getJW_setInter:"",//得经纬度的计时器函数的编号
    ifshowMarkers:false,
    markers: [{
      iconPath: '../../images/ditudingweidian-xuanzhong.png',
      id: 0,
      latitude: "30.96174659643712", 
      longitude: "116.35552389252058", 
      width:20,
      height: 35
    }],
    mode: 0,
  },

  /**
   * 生命周期函数--监听页面加载
   */
  onLoad: function (options) {
    var page = this;
    wx.getSystemInfo({
      success: function (res) {
        console.log(res);
        page.setData({ winWidth: res.windowWidth });
        page.setData({ winHeight: res.windowHeight });
      },
    })
  },

  /**
   * 生命周期函数--监听页面初次渲染完成
   */
  onReady: function () {
    this.getDeviceInfo()
  },
  bindchange: function (e) {
    const that = this;
    that.setData({
      currentData: e.detail.current
    })
  },
  getDeviceInfo: function () {
    let that = this
    wx.getSystemInfo({
      success: function (res) {
        that.setData({
          deviceW: res.windowWidth,
          deviceH: res.windowHeight
        })
      }
    })
  },
  
  gps_data: function () {
    let that = this;
    //从oneNET请求我们的Wi-Fi气象站的数据
    const requestTask = wx.request({
      url: 'https://api.heclouds.com/devices/639504816/datapoints?datastream_id=latitude,longitude&limit=10',
      header: {
        'content-type': 'application/json',
        'api-key': 'fK=PEyZjPraXWFdfQqfR2K88tEU='
      },
      success: function (res) {
        //console.log(res.data)
        //拿到数据后保存到全局数据
        var app = getApp()
        app.globalData.latitude = res.data.data.datastreams[0]
        app.globalData.longitude = res.data.data.datastreams[1]
        console.log(app.globalData.longitude)
        console.log(app.globalData.latitude)
        that.setData({
          markers: [{
            iconPath: '../../images/ditudingweidian-xuanzhong.png',
            id: 0,
            latitude: res.data.data.datastreams[0],
            longitude: res.data.data.datastreams[1],
            width: 20,
            height: 35
          }],
          car_latitude: res.data.data.datastreams[0],
          car_longitude: res.data.data.datastreams[1],
        })
      },

      fail: function (res) {
        console.log("fail!!!")
      },

      complete: function (res) {
        console.log("end")
      }
    })
  },

  formSubmit: function (e) {
    var that = this;
    console.log('form发生了submit事件，携带数据为：', e.detail.value)
    wx.request({
      url: 'https://api.heclouds.com/devices/639504816/datapoints?type=3',
      header: {
        'api-key': 'fK=PEyZjPraXWFdfQqfR2K88tEU='
      },
      method: "POST",
      data: {
        'mode': this.data.mode
      },
      success: function (res) {
        // 操作json数据
        //console.log(res.statusCode);        
        if (res.statusCode == 200)
          that.setData({ motto: "mode change" })
        else
          that.setData({ motto: "404 not found" })
      }
    })    
  },
  formReset: function () {
    console.log('form发生了reset事件')
  },
  bindTimeChange: function (e) {
    // console.log('picker发送选择改变，携带值为', e.detail.value)
    this.setData({
      time: e.detail.value
    })
  },
 
  show: function (e) {
    console.log('radio模式选择进行了操作', e)
    console.log(e.detail.value)
    if (e.detail.value == "enroll") {
      this.setData({
        showStep: false,
      })
    }
    if (e.detail.value == "tour") {
      this.setData({
        showStep: true,
        mode: 4
      })
    }
    console.log(this.data.mode)
  },
  show1: function (e) {
    console.log('radio模式选择进行了操作', e)
    console.log(e.detail.value)
    if (e.detail.value == "1") {
      this.setData({
        mode: 1
      })
    }
    if (e.detail.value == "2") {
      this.setData({
        mode: 2
      })
    }
    if (e.detail.value == "3") {
      this.setData({
        mode: 3
      })
    }
    console.log(this.data.mode)
  },
  checkCurrent: function (e) {
    const that = this;

    if (that.data.currentData === e.target.dataset.current) {
      return false;
    } else {

      that.setData({
        currentData: e.target.dataset.current
      })
    }
  },
  getJ_W: function () {
    let that = this;
    let alongitude="";
    let alatitude="";
    //将计时器赋值给setInter
    that.data.getJW_setInter = setInterval(
      function () {
        const requestTask =wx.request({
         url: 'https://api.heclouds.com/devices/639504816/datapoints?datastream_id=latitude,longitude&limit=1',
         header: {
           'content-type': 'application/json',
           'api-key': 'fK=PEyZjPraXWFdfQqfR2K88tEU='
         },
          success: function (res) {
            console.log(res.data.data)
            console.log("res.data.latitude=" + res.data.data.datastreams[0].datapoints[0].value)
            console.log("res.data.longitude=" + res.data.data.datastreams[1].datapoints[0].value)
            // alatitude = res.data.data.datastreams[0].datapoints[0].value-0.005,
            // alongitude = res.data.data.datastreams[1].datapoints[0].value-0.00703;
            alatitude = res.data.data.datastreams[0].datapoints[0].value,
              alongitude = res.data.data.datastreams[1].datapoints[0].value;
              var result = [alongitude, alatitude]
            // var result = gcoord.transform(
            //   [alongitude, alatitude],    // 经纬度坐标
            //   gcoord.BMap,                 // 当前坐标系 硬件GPS
            //   gcoord.GCJ02);
           // var result = [alongitude, alatitude]  // 经纬度坐标
            console.log("result=" + result)
           that.setData({
             markers: [{
               iconPath: '../../images/ditudingweidian-xuanzhong.png',
               id: 0,
               latitude: result[1],
               longitude: result[0],
               width: 20,
               height: 35
             }],
            
            
           })
            console.log("marker.longitude="+that.data.markers[0].longitude)
            console.log("marker.alatitude=" + that.data.markers[0].latitude)
          }
        })
        // console.log("每隔5s获得一次经纬度")
        // //发送给刘组
        // var send_result = gcoord.transform(
        //   [alongitude, alatitude],    // 经纬度坐标
        //   gcoord.WGS84,                 // 当前坐标系 硬件GPS
        //   gcoord.BD09);
        // wx.request({
        //   url: 'http://182.92.86.34:8099/apis/sendpos',
        //   method: 'GET',
        //   data:{
        //     latitude: send_result[1],
        //     longitude: send_result[0], 
        //   },
        //   success: function (res) {
        //     console.log("发送给合作组"+JSON.stringify(res))
        //   }
        // })
      }
      , 10000); 
  },
  //取消获得经纬度的重复函数
  cancel_get_J_W:function(){
    let that=this;
    clearInterval(that.data.getJW_setInter)
  },
  ClickOn: function () {
    /* if (app.globalData.userInfo.nickName != '李行')
       return;*/
     var that=this;
     this.setData({ motto: "wait..." });
     wx.request({
       url: 'https://api.heclouds.com/devices/639504816/datapoints?type=3',
       header:{
         'api-key': 'fK=PEyZjPraXWFdfQqfR2K88tEU='
       },
       method:"POST",
       data: {
         'led_pi': '1'
       },
       success: function (res) {
         // 操作json数据
         //console.log(res.statusCode);        
         if(res.statusCode==200)
           that.setData({ motto: "led on" })
         else
           that.setData({ motto: "404 not found" })
       }
     })    
     
   },
   ClickOff: function () {
     /*if (app.globalData.userInfo.nickName != '李行')
       return;*/
     var that=this;
     this.setData({ motto: "wait..." })
     wx.request({
       url: 'https://api.heclouds.com/devices/639504816/datapoints?type=3',
       header: {
         'api-key': 'fK=PEyZjPraXWFdfQqfR2K88tEU='
       },
       method: "POST",
       data: {
         'led_pi': '0'
       },
       success: function (res) {
         // 操作json数据
         //console.log(res.statusCode);
         if (res.statusCode == 200)
           that.setData({ motto: "led off" })
         else
           that.setData({ motto: "404 not found" })
       }
     })
   },
   ClickGet: function () {
     this.setData({ motto: "wait" })
     wx.request({
       url: 'https://api.heclouds.com/devices/639504816/datapoints?datastream_id=led_pi',
       header: {
         'api-key': 'fK=PEyZjPraXWFdfQqfR2K88tEU='
       },
       method: "GET",      
       success(res) {
         console.log(res.data)
       }
     })
     
   },
  getCarstate:function()
  { let that=this;
    wx.request({
      url: 'http://139.199.105.136:6888/car/request',
      method: 'GET',
      success: function (res) {
        if (res.data.car_state=="c")
        {
          that.setData({
            ifArrive: true
          })
        }
       else{
          that.setData({
            ifArrive: false
          })
       }
        console.log(res.data.car_state)
      }
    })
 
  },
  /**
   * 生命周期函数--监听页面显示
   */
  onShow: function () {

  },

  /**
   * 生命周期函数--监听页面隐藏
   */
  onHide: function () {

  },

  /**
   * 生命周期函数--监听页面卸载
   */
  onUnload: function () {

  },

  /**
   * 页面相关事件处理函数--监听用户下拉动作
   */
  onPullDownRefresh: function () {

  },

  /**
   * 页面上拉触底事件的处理函数
   */
  onReachBottom: function () {

  },

  /**
   * 用户点击右上角分享
   */
  onShareAppMessage: function () {

  }
})