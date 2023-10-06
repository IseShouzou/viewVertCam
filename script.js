var sFn = new SimFunc();

onload = function(){

    document.oncontextmenu = function () {return false;}

    let sliderA = document.getElementById( "SliderAlpha" );
    let sliderB = document.getElementById( "SliderBeta"  );
    let sliderC = document.getElementById( "SliderGamma" );

    let alp = sliderA.value;
    let bet = sliderB.value;
    let gam = sliderC.value;

    sliderA.addEventListener("input", update );
    sliderB.addEventListener("input", update );
    sliderC.addEventListener("input", update );

    window.addEventListener("deviceorientation", function(e){
        alp = ( e.alpha || 0);
        bet = ( e.beta  || 0);
        gam = ( e.gamma || 0);
        update();
    });


    let canvas1 = document.getElementById('canvas');
    let view1 = new SceneView( canvas1 );

    let canvas2 = document.getElementById("canvas2");
    let ctx = canvas2.getContext("2d");

    canvas2.setAttribute("width", screen.width );
    canvas2.setAttribute("height", screen.height );


    let fovy = 90.0;

    let pnts = [ [ -0.2,  0.5, -0.5 ],
                 [  0.2,  0.5, -0.5 ],
                 [  0.2,  0.5, -0.2 ],
                 [ -0.2,  0.5, -0.2 ] ];



    window.addEventListener( 'resize', ()=>{
        console.log( 'resize Event' );

        resize();

} )


    function resize(){
        //console.log( 'resize' );

        view1.resize();

        canvas2.setAttribute("width", screen.width );
        canvas2.setAttribute("height", screen.height );

        //console.log( 'canvas1 : ',canvas1.width, canvas1.height  )
        //console.log( 'canvas2 : ',canvas2.width, canvas2.height  )
        //console.log( 'screen  : ', screen.width, screen.height  )

    }


    function update(){
        //console.log( 'update' );

        //alp = sliderA.value;
        //bet = sliderB.value;
        //gam = sliderC.value;

        view1.alp = alp;
        view1.bet = bet;
        view1.gam = gam;

        view1.update();

    }


    function xx(){
        update();
        resize();
    }
    setTimeout(xx, 100);



    function calcDCM(){

        let alpRad = alp * Math.PI / 180.0;
        let betRad = bet * Math.PI / 180.0;
        let gamRad = gam * Math.PI / 180.0

        let ca = Math.cos( alpRad );
        let sa = Math.sin( alpRad );
        let cb = Math.cos( betRad );
        let sb = Math.sin( betRad );
        let cg = Math.cos( gamRad );
        let sg = Math.sin( gamRad );

        let m11 =   cg * ca - sg * sb * sa;
        let m21 = - cb * sa ;
        let m31 =   sg * ca + cg * sb * sa;

        let m12 =  cg * sa + sg * sb * ca;
        let m22 =  cb * ca ;
        let m32 =  sg * sa - cg * sb * ca ;

        let m13 = - sg * cb;
        let m23 =   sb;
        let m33 =   cg * cb;

        // vertical
        //let mat = [ [ -m32, -m31,  m33 ],
        //            [  m12,  m11, -m13 ],
        //            [ -m22, -m21,  m23 ] ];
        // horizontal
        //let mat = [ [ -m31, m32, m33 ],
        //            [ -m21, m22, m23 ],
        //            [ -m11, m12, m13 ] ];

        let mat = [ [ -m31, -m32, -m33 ],
                    [  m11,  m12,  m13 ],
                    [ -m21, -m22, -m23 ] ];

        phi = 180.0 / Math.PI * Math.atan2(   mat[1][2], mat[2][2] );
        the = 180.0 / Math.PI * Math.atan2( - mat[0][2], Math.sqrt( mat[1][2] * mat[1][2] + mat[2][2] * mat[2][2]) );
        psi = 180.0 / Math.PI * Math.atan2(   mat[0][1], mat[0][0] );

        return mat;

    }


    let count = 0;

    navigator.mediaDevices
        .getUserMedia({ audio: false, video: { facingMode: "environment" } })
        .then(stream => {
            let imageCapture = new ImageCapture( stream.getVideoTracks()[0] );

            setInterval(() => {

                imageCapture.grabFrame()
                    .then((imageBitmap) => {

                        count += 1;
                        //console.log('XXX');

                        let W = canvas.width;
                        let H = canvas.height;

                        //console.log( 'W, H  :  ',W, H );

                        let e = H / 2.0 / Math.tan( fovy / 2.0 * Math.PI / 180.0 );
                        let dcm = calcDCM();
                        let coeff = [ [   0.0        , - 2.0 * e / H ],
                                      [   0.0        ,   2.0 * e / H ],
                                      [ - 2.0 * e / W,   0.0         ],
                                      [   2.0 * e / W,   0.0         ] ];

                        let curPnts = [];
                        for( var p of pnts ){
                            curPnts.push( [ 0.0,  0.0,  0.0 ] );
                            for( let i=0; i<3; i++ ){
                                for( let j=0; j<3; j++ ){
                                    curPnts.slice(-1)[0][i] += dcm[i][j] * p[j];
                                }
                            }
                        }

                        for( var c of coeff ){
                            if( curPnts.length < 1 ) break;
                            var newPnts = [];
                            var PA = curPnts.slice(-1)[0];
                            var SA = PA[0] + c[0] * PA[1]  + c[1] * PA[2];
                            for( var p of curPnts ){
                                var PB = p;
                                var SB = PB[0] + c[0] * PB[1]  + c[1] * PB[2];
                                if( SB >= 0.0 ){
                                    if( SA <= 0.0 ){
                                        var ka =   SB / ( SB - SA );
                                        var kb = - SA / ( SB - SA );
                                        newPnts.push( [ ka * PA[0] + kb * PB[0],
                                                        ka * PA[1] + kb * PB[1],
                                                        ka * PA[2] + kb * PB[2]  ] );
                                    }
                                    newPnts.push( PB );
                                }else{
                                    if( SA > 0.0 ){
                                        var ka = - SB / ( SA - SB );
                                        var kb =   SA / ( SA - SB );
                                        newPnts.push( [ ka * PA[0] + kb * PB[0],
                                            ka * PA[1] + kb * PB[1],
                                            ka * PA[2] + kb * PB[2]  ] );
                                    }
                                }
                                PA = PB;
                                SA = SB;
                            }

                            curPnts = newPnts;
                        }

                        ctx.save();

                        ctx.clearRect( 0, 0, W, H )

                        //ctx.fillStyle = "#ddd";
                        //ctx.fillRect(0, 0, W, H );

                        ctx.beginPath();

                        flag = true;
                        for( let p of curPnts ){

                            XX = W / 2.0 + e * p[1] / p[0];
                            YY = H / 2.0 + e * p[2] / p[0];

                            if( flag ){ ctx.moveTo( XX, YY ); }
                            else      { ctx.lineTo( XX, YY ); }
                            flag = false;
                        }

                        ctx.closePath();
                        ctx.stroke();
                        ctx.clip();

                        wb = imageBitmap.width;
                        hb = imageBitmap.height;
                        if ( W / H < wb / hb ){
                            let w =  W / H * hb;
                            let x = ( wb - w ) / 2;
                            ctx.drawImage(imageBitmap, x, 0, w, hb, 0, 0 ,W, H );
                        } else {
                            let h =  H / W * wb;
                            let y = ( hb - h ) / 2;
                            ctx.drawImage(imageBitmap, 0, y, wb, h, 0, 0 ,W, H );
                        }

                        ctx.restore();

                    })
                    .catch( (e) => {} );
             }, 10 );

        })
        .catch(e => alert("error" + e.message));




}



function SceneView( canvas ){

    this.canvas = canvas;
    this.glView = new GlView( canvas );

    var glView = this.glView;

    canvas.width = canvas.clientWidth;
    canvas.height = canvas.clientHeight;

    glView.numViews = 1;
    glView.viewFov  = [ 90 ];

    glView.viewport = [ [ 0, 0, canvas.width, canvas.height ] ];

    //let x = 0.1 * glView.canvas.width
    //let y = 0.1 * glView.canvas.height
    //let w = 0.8 * glView.canvas.width
    //let h = 0.8 * glView.canvas.height
    //glView.viewport = [ [ x, y, w, h ] ];


    var e = [     0.0,     0.0, 5000.0 ];
    var t = [  9000.0,  9000.0, 5000.0 ];
    var u = [ 0.0, 0.0, 1.0 ];
    glView.lookAtMat = [ calcLookAt( e, t, u ) ];

    glView.viewMat = new Array( this.numViews );
    glView.projMat = new Array( this.numViews );


    //
    //-------------------
    //    地形
    //-------------------

    glView.addGeomObje( 'geom' );

    var img = new Image();
    img.onload = function(){
        glView.mkGeomTexture(img);
    }
    img.src = getImageData();



    //
    //-------------------
    //    空港
    //-------------------

    var color =  [ 0.1, 0.1, 0.1,1.0,  0.5, 0.5, 0.0,1.0,  0.0, 0.0, 0.0, 1.0, 50]
    glView.addRectObje( 'airport', color, [ 3200, 200, 1 ] , [ 0, 0, 0.01] );

    var color =  [ 0.1, 0.1, 0.1,1.0,  0.5, 0.5, 0.5,1.0,  0.0, 0.0, 0.0, 1.0, 50]
    glView.addRectObje( 'airport', color, [ 3000, 60, 1 ] , [ 0, 0, 0.02] );

    var color =  [ 0.1, 0.1, 0.1,1.0,  1.0, 1.0, 1.0,1.0,  0.0, 0.0, 0.0, 1.0, 50]

    for( k=-1; k<2; k++){
        glView.addRectObje( 'airport', color, [ 1.8, 60.0, 0.5 ] , [ -3.6*k, 0.0, 0.03] );
    }

    for( k=-28; k<28; k++){
        glView.addRectObje( 'airport', color, [ 30.0, 2.0, 0.5 ] , [ 50.0*k+25.0, 0.0, 0.03] );
    }

    for( k=1; k<9; k++){
        glView.addRectObje( 'airport', color, [ 30.0, 1.8, 0.5 ] , [ 1479,  k*3.6, 0.03] );
        glView.addRectObje( 'airport', color, [ 30.0, 1.8, 0.5 ] , [-1479,  k*3.6, 0.03] );
        glView.addRectObje( 'airport', color, [ 30.0, 1.8, 0.5 ] , [ 1479, -k*3.6, 0.03] );
        glView.addRectObje( 'airport', color, [ 30.0, 1.8, 0.5 ] , [-1479, -k*3.6, 0.03] );
    }

    glView.setPosByName( 'airport', 0, 10000, 600 );


}

SceneView.prototype.resize = function(){
    //console.log( 'resize SceneView' );

    //console.log( this.canvas.clientWidth, this.canvas.clientHeight );
    //console.log( this.glView.viewport );
    this.canvas.width = this.canvas.clientWidth;
    this.canvas.height = this.canvas.clientHeight;

    //console.log( this.canvas.width, this.canvas.height );


    this.glView.viewport = [ [ 0, 0, this.canvas.width, this.canvas.height ] ];

    //let x = 0.1 * this.canvas.width
    //let y = 0.1 * this.canvas.height
    //let w = 0.8 * this.canvas.width
    //let h = 0.8 * this.canvas.height
    //this.glView.viewport = [ [ x, y, w, h ] ];

    this.update();

}


SceneView.prototype.update = function(){

    //console.log('update SceneView');
    //console.log( this.alp, this.bet, this.gam );


    let glView = this.glView;

    let invlookAtMat = invLookAtMat( glView.lookAtMat[0] );
    let xyz = [ invlookAtMat[3], invlookAtMat[7], invlookAtMat[11]  ];

    //console.log( xyz );
    //console.log( invlookAtMat );
    //console.log( invlookAtMat[3] );
    //console.log( this.alp, this.bet, this.gam );

    let m = calcMat( this.alp, this.bet, this.gam );

    let mat = [ [   m[1][0],   m[1][1],   m[1][2] ],
                [ - m[2][0], - m[2][1], - m[2][2] ],
                [ - m[0][0], - m[0][1], - m[0][2] ] ];

    glView.lookAtMat[0] = calcLookAtfromMat33XYZ( mat, xyz );

    glView.update();

}


function calcMat( alp, bet, gam ){

        let alpRad = alp * Math.PI / 180.0;
        let betRad = bet * Math.PI / 180.0;
        let gamRad = gam * Math.PI / 180.0

        let ca = Math.cos( alpRad );
        let sa = Math.sin( alpRad );
        let cb = Math.cos( betRad );
        let sb = Math.sin( betRad );
        let cg = Math.cos( gamRad );
        let sg = Math.sin( gamRad );

        let m11 =   cg * ca - sg * sb * sa;
        let m21 = - cb * sa ;
        let m31 =   sg * ca + cg * sb * sa;

        let m12 =  cg * sa + sg * sb * ca;
        let m22 =  cb * ca ;
        let m32 =  sg * sa - cg * sb * ca ;

        let m13 = - sg * cb;
        let m23 =   sb;
        let m33 =   cg * cb;

        // vertical
        //let mat = [ [ -m32, -m31,  m33 ],
        //            [  m12,  m11, -m13 ],
        //            [ -m22, -m21,  m23 ] ];

        // horizontal
        //let mat = [ [ -m31, m32, m33 ],
        //            [ -m21, m22, m23 ],
        //            [ -m11, m12, m13 ] ];

        let mat = [ [ -m31, -m32, -m33 ],
                    [  m11,  m12,  m13 ],
                    [ -m21, -m22, -m23 ] ];

        phi = 180.0 / Math.PI * Math.atan2(   mat[1][2], mat[2][2] );
        the = 180.0 / Math.PI * Math.atan2( - mat[0][2], Math.sqrt( mat[1][2] * mat[1][2] + mat[2][2] * mat[2][2]) );
        psi = 180.0 / Math.PI * Math.atan2(   mat[0][1], mat[0][0] );

        //console.log( 'abg : ', alp, bet, gam )
        //console.log( 'eul : ', Math.round( phi ), Math.round( the ), Math.round( psi ) )

        return mat;

}


//--------------------------------------------------------
//
//	数学関数
//
//--------------------------------------------------------

function identity(){
    return new Float32Array([ 1, 0, 0, 0,
                              0, 1, 0 ,0,
                              0, 0, 1, 0,
                              0, 0, 0, 1 ]);
}


function normalize(x){
    var y = x.slice();
    var v = 0.0;
    for (var i=0; i<x.length; i++) {
        v += x[i] * x[i];
    }
    v = Math.sqrt( v );
    for (var i=0; i<x.length; i++) {
        y[i] = x[i] / v;
    }
    return y;
}


function cross(x,y){
    var z = x.slice();
    z[0] = x[1]*y[2] - x[2]*y[1];
    z[1] = x[2]*y[0] - x[0]*y[2];
    z[2] = x[0]*y[1] - x[1]*y[0];
    return z;
}


//    0   1   2   3    
//    4   5   6   7    
//    8   9  10  11    
//   12  13  14  15    


function transpose(x){
    var y = x.slice();
    for (var i=0; i<4; i++) { //>
        for (var j=0; j<4; j++) { //>
            y[4*i+j] = x[i+4*j];
        }
    }
    return y;
}


function productMat(x,y){
    var z = x.slice();
    for (var i=0; i<16; i+=4) { //>
        for (var j=0; j<4; j++) { //>
            z[i+j] = 0.0;
            for (var k=0; k<4; k++) { //>
                z[i+j] += x[i+k] * y[j+4*k]; 
            }
        }
    }
    return z;
}


function productMV(x,y){
    var z = y.slice();
    for (var i=0; i<4; i++) { //>
        z[i] = 0.0;
        for (var j=0; j<4; j++) { //>
            z[i] += x[4*i+j] * y[j];
        }
    }
    return z;
}



function calcDCM( euler, flag = false ){

    var ang1 = euler[0] * Math.PI / 180.0;
    var ang2 = euler[1] * Math.PI / 180.0;
    var ang3 = euler[2] * Math.PI / 180.0;

    if ( flag ) {
        ang2 = - ang2;
        ang3 = - ang3;
    }

    var [  c1 ,  s1  ] = [ Math.cos( ang1 ), Math.sin( ang1 ) ];
    var [  c2 ,  s2  ] = [ Math.cos( ang2 ), Math.sin( ang2 ) ];
    var [  c3 ,  s3  ] = [ Math.cos( ang3 ), Math.sin( ang3 ) ];

    return [ c2 * c3,                 c2 * s3,                - s2,    0.0,
             s1 * s2 * c3 - c2 * s3,  s1 * s2 * s3 + c1 * c3, s1 * c2, 0.0,
             c1 * s2 * c3 + s2 * s3,  c1 * s2 * s3 - s1 * c3, c1 * c2, 0.0,
             0.0,                     0.0,                    0.0,     1.0, ];

}

//    0   1   2   3    
//    4   5   6   7    
//    8   9  10  11    
//   12  13  14  15    



function calcLookAtfromMat33XYZ( mat, xyz ){

    X = - ( mat[0][0] * xyz[0] + mat[0][1] * xyz[1] + mat[0][2] * xyz[2] );
    Y = - ( mat[1][0] * xyz[0] + mat[1][1] * xyz[1] + mat[1][2] * xyz[2] );
    Z = - ( mat[2][0] * xyz[0] + mat[2][1] * xyz[1] + mat[2][2] * xyz[2] );

    return [ mat[0][0], mat[0][1], mat[0][2], X   ,
             mat[1][0], mat[1][1], mat[1][2], Y   ,
             mat[2][0], mat[2][1], mat[2][2], Z   ,
             0.0      , 0.0      , 0.0      , 1.0  ];

}



function calcLookAt( e, t, u ){

    var z = normalize( [ e[0] - t[0], e[1] - t[1], e[2] - t[2] ] );
    var x = normalize( cross( u, z ) );
    var y = normalize( cross( z, x ) );;

    return [ x[0], x[1], x[2], - e[0] * x[0] - e[1] * x[1] - e[2] * x[2] ,
             y[0], y[1], y[2], - e[0] * y[0] - e[1] * y[1] - e[2] * y[2] ,
             z[0], z[1], z[2], - e[0] * z[0] - e[1] * z[1] - e[2] * z[2] ,
              0.0,  0.0,  0.0, 1.0 ];
}

//    0   1   2   3    
//    4   5   6   7    
//    8   9  10  11    
//   12  13  14  15    


function invLookAtMat(x){
    var y = x.slice();
    y[ 1] =  x[ 4];
    y[ 2] =  x[ 8];
    y[ 4] =  x[ 1];
    y[ 6] =  x[ 9];
    y[ 8] =  x[ 2];
    y[ 9] =  x[ 6];
    y[ 3] = -y[ 0] * x[ 3] - y[ 1] * x[ 7] - y[ 2] * x[11];
    y[ 7] = -y[ 4] * x[ 3] - y[ 5] * x[ 7] - y[ 6] * x[11];
    y[11] = -y[ 8] * x[ 3] - y[ 9] * x[ 7] - y[10] * x[11];
    return y
}



//    0   1   2   3 
//    4   5   6   7 
//    8   9  10  11 
//   12  13  14  15 

function orthogonalMatrix( w, asp, near, far ){

    var [ xs, dx ] = [ 0          , w          ] ;
    var [ ys, dy ] = [ 0          , w / asp    ] ;
    var [ zs, dz ] = [ far + near , far - near ] ;
    var mat = identity() ;
    mat[ 0] =   2.0 / dx ; //00
    mat[ 5] =   2.0 / dy ; //11
    mat[10] = - 2.0 / dz ; //22
    mat[ 3] = - xs / dx  ; //31
    mat[ 7] = - ys / dy  ; //32
    mat[11] = - zs / dz  ; //33

    return mat;
}



function perspectiveMatrix( fov, asp, near, far, flagCot = false ){

    if ( flagCot ) {
        var cot = fov;
    }else{
        var cot = 1.0 / Math.tan( Math.PI / 180 * fov / 2.0 );
    }

    var mat = identity();
    mat[ 0] = cot / asp                       ; //00
    mat[ 5] = cot                             ; //11
    //mat[10] = - (far+near) / (far-near)       ; //22
    mat[14] = -1.0                            ; //32
    //mat[11] = - 2.0 * far * near / (far-near) ; //23
    mat[15] =  0.0 ; //33

    //mat[10] = - ( 0.9*far+near) / (far-near)       ; //22
    //mat[11] = - 1.9 * far * near / (far-near) ; //23

    //console.log( far,near );

    mat[10] = - 2.0 / (far-near)       ; //22
    mat[11] = - (far+near) / (far-near) ; //23

    return mat;
}


function invViewMat(x){
    var y = x.slice();
    y[ 0] = 1.0 / x[ 0];
    y[ 5] = 1.0 / x[ 5];
    D = x[10]*x[15] - x[11]*x[14];
    y[10] =  x[15] / D;
    y[15] =  x[10] / D;
    y[11] = -x[11] / D;
    y[14] = -x[14] / D;
    return y
}

//    0   1   2   3 
//    4   5   6   7 
//    8   9  10  11 
//   12  13  14  15 




function compare(a, b){
    return [ [ Math.min( a[0][0], b[0][0] ),
               Math.min( a[0][1], b[0][1] ),
               Math.min( a[0][2], b[0][2] ),
               1.0 ],
             [ Math.max( a[1][0], b[1][0] ),
               Math.max( a[1][1], b[1][1] ),
               Math.max( a[1][2], b[1][2] ),
               1.0 ] ];
}



function calcLightProjMat(eye, MinMax){

    var t = [ ( MinMax[0][0] +  MinMax[1][0] ) / 2.0,
              ( MinMax[0][1] +  MinMax[1][1] ) / 2.0,
              ( MinMax[0][2] +  MinMax[1][2] ) / 2.0  ];

    var k = 0;
    var u = [ 1.0, 0.0, 0.0 ];
    for( var i=0; i<3; i++ ){
        if ( Math.abs( t[i] - eye[i] ) < Math.abs( t[k] - eye[k] ) ){
            u[i] = 1.0; u[k] = 0.0;
        }
    }
    var LKAT = calcLookAt(eye, t, u);

    var tn = 0;
    var near; var far; 
    for(var i=0;i<8;i++) {
        var xyz = productMV( LKAT,
                             Array( MinMax[ i                 % 2 ][0],
                                    MinMax[ Math.floor( i/2 ) % 2 ][1],
                                    MinMax[ Math.floor( i/4 ) % 2 ][2],
                                    1.0 ) );

        if( Math.abs( xyz[2] ) > 0.0001){
            tn = Math.max( tn, Math.abs( xyz[0] / xyz[2] ) );
            tn = Math.max( tn, Math.abs( xyz[1] / xyz[2] ) );
        }
        if( i==0 ) {
            near = - xyz[2];
            far  = - xyz[2];
        }else{
            near = Math.min( near, - xyz[2] );
            far  = Math.max( far , - xyz[2] );
        }
    }
    tn *= 1.05

    if( far < 0.0 ){
        near = 0.1;
        far  = 1.0;
    }else if( near < 0.001 ){
        near = 0.001;
    }else{
        var d = Math.max( 0.001, 0.1 * ( far - near ) );
        far  += d;
        near -= d;
        //far = far * 1.1 - 0.1 * near
        //near *= 0.9;
    }


    //console.log(near, far); 
    //console.log(tn); 
    //console.log( perspectiveMatrix(1 / tn, 1, near, far, true) ); 
    //console.log( LKAT ); 

    return productMat( perspectiveMatrix(1 / tn, 1, near, far, true), LKAT);

}

function calcPilotLookAtMat(vPosMat, pPos, pEul){

    var dcm = transpose( calcDCM( pEul, true ) );
    dcm[ 3] = pPos[0] ;
    dcm[ 7] = pPos[1] ;
    dcm[11] = pPos[2] ;

    mat = invLookAtMat( productMat( vPosMat, dcm ) );

    //mat =  productMat( invLookAtMat( vPosMat ), dcm );

    return [ - mat[ 4], - mat[ 5], - mat[ 6], - mat[ 7],
               mat[ 8],   mat[ 9],   mat[10],   mat[11],
             - mat[ 0], - mat[ 1], - mat[ 2], - mat[ 3],
               mat[12],   mat[13],   mat[14],   mat[15] ];

}


function calcChaserLookAtMat(vPosMat, cPos ){

    if( Math.abs( cPos[0] ) + Math.abs( cPos[1] ) + Math.abs( cPos[2] )  < 0.001 ){
        cPos = [ 0.001, 0.001, 0.001 ] ;
    }

    var t = [ vPosMat[ 3], vPosMat[ 7], vPosMat[11] ];
    var e = [ t[0] + cPos[0] , t[1] + cPos[1] , t[2] + cPos[2]  ];
    var u = [ 0.0, 0.0, 1.0 ];

    return calcLookAt( e, t, u );

}




function arrayBufferToBase64(buffer) {
  let binary = '';
  const bytes = new Uint8Array(buffer);
  const len = bytes.byteLength;
  for (let i = 0; i < len; i++) {
    binary += String.fromCharCode(bytes[i]);
  }
  return window.btoa(binary);
}



function base64ToArrayBuffer(base64) {
  const binary_string = window.atob(base64);
  const len = binary_string.length;
  let bytes = new Uint8Array(len);
  for (let i = 0; i < len; i++) {
    bytes[i] = binary_string.charCodeAt(i);
  }
  return bytes.buffer;
}


