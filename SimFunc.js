


function SimFunc(){


    this.add = function( ...x ){
        return x.reduce( (a,e) => a.map( (v,i) => v + e[i] ) );
    }


    this.sub = function( ...x ){
        return x.reduce( (a,e) => a.map( (v,i) => v - e[i] ) );
    }


    this.mults = function( ...x ){
        return x.reduce( (a,e) => this.mult(a,e) );
    }

    this.mult = function( x, y, n=-1 ){

        if( !Array.isArray(x) ) return y.map( v => x * v );
        if( !Array.isArray(y) ) return x.map( v => y * v );
        if( n==0 )              return x.map( (v,i) => v * y[i] );

        if( n==-1 ){

            if( x.length == y.length ){
                n = Math.sqrt( x.length );
                if ( ! Number.isInteger( n ) ) n = x.length;
            }else{
                n = Math.max( x.length, y.length ) / Math.min( x.length, y.length );
            }

            //n = ( x.length == y.length ) ? Math.sqrt( x.length ) :
            //     Math.max( x.length, y.length ) / Math.min( x.length, y.length );
        }

        var p = x.length / n;
        var q = y.length / n;
        var z = new Array( p * q );
        for( i=0; i<p; i++){
            for( j=0; j<q; j++){
                z[ q*i+j ] = 0.0;
                for( k=0; k<n; k++){
                    z[ q*i+j ] += x[ n*i+k ] * y[ q*k+j ];
                }
            }
        }
        return z;

    }


    this.cross = function( a, b ){
        return [ a[1]*b[2] -  a[2]*b[1],
                 a[2]*b[0] -  a[0]*b[2],
                 a[0]*b[1] -  a[1]*b[0] ];
    }

    this.tr3 = function( m ){
        return [ m[0], m[3], m[6],
                 m[1], m[4], m[7],
                 m[2], m[5], m[8] ];
    }


    this.tilde = function( m ){
        return [   0.0 , - m[2],    m[1],
                   m[2],   0.0 ,  - m[0],
                 - m[1],   m[0],    0.0  ];
    }

    //
    //  0  1  2
    //  3  4  5
    //  6  7  8
    //
    this.invMat = function( m ){
    //console.log('invMat');

        var det = m[0]*m[4]*m[8] + m[2]*m[3]*m[7] + m[6]*m[1]*m[5]
                - m[2]*m[4]*m[6] - m[0]*m[5]*m[7] - m[8]*m[1]*m[3];

        return [ ( m[4]*m[8] - m[5]*m[7] ) / det,
                 ( m[2]*m[7] - m[1]*m[8] ) / det,
                 ( m[1]*m[5] - m[2]*m[4] ) / det,
                 ( m[5]*m[6] - m[3]*m[8] ) / det,
                 ( m[0]*m[8] - m[2]*m[6] ) / det,
                 ( m[2]*m[3] - m[0]*m[5] ) / det,
                 ( m[3]*m[7] - m[4]*m[6] ) / det,
                 ( m[1]*m[6] - m[0]*m[7] ) / det,
                 ( m[0]*m[4] - m[1]*m[3] ) / det ];

    }


    this.euler2dcm = function( e ){

        var [ c1, s1 ] = [ Math.cos( e[0] ), Math.sin( e[0] ) ];
        var [ c2, s2 ] = [ Math.cos( e[1] ), Math.sin( e[1] ) ];
        var [ c3, s3 ] = [ Math.cos( e[2] ), Math.sin( e[2] ) ];

        return [ c2 * c3               , c2 * s3               , -s2     , 
                 s1 * s2 * c3 - c1 * s3, s1 * s2 * s3 + c1 * c3,  s1 * c2,
                 c1 * s2 * c3 + s1 * s3, c1 * s2 * s3 - s1 * c3,  c1 * c2 ];

    }


    this.dcm2quat = function( dcm ){

        var q = [ Math.sqrt(1 + dcm[0] - dcm[4] - dcm[8] ) / 2.0,
                  Math.sqrt(1 - dcm[0] + dcm[4] - dcm[8] ) / 2.0,
                  Math.sqrt(1 - dcm[0] - dcm[4] + dcm[8] ) / 2.0,
                  Math.sqrt(1 + dcm[0] + dcm[4] + dcm[8] ) / 2.0 ]

        var ix = 0;
        for( var i=1; i<4; i++){
            if( q[i] > q[ix] ) ix = i;
        }

        if( ix == 0 ){
            q[1] = 0.25 / q[0] * ( dcm[1] + dcm[3] );
            q[2] = 0.25 / q[0] * ( dcm[2] + dcm[6] );
            q[3] = 0.25 / q[0] * ( dcm[5] - dcm[7] );
        }else if( ix == 1 ){
            q[0] = 0.25 / q[1] * ( dcm[1] + dcm[3] );
            q[2] = 0.25 / q[1] * ( dcm[7] + dcm[5] );
            q[3] = 0.25 / q[1] * ( dcm[6] - dcm[2] );
        }else if( ix == 2 ){
            q[0] = 0.25 / q[2] * ( dcm[6] + dcm[2] );
            q[1] = 0.25 / q[2] * ( dcm[7] + dcm[5] );
            q[3] = 0.25 / q[2] * ( dcm[1] - dcm[3] );
        }else if( ix == 3 ){
            q[0] = 0.25 / q[3] * ( dcm[5] - dcm[7] );
            q[1] = 0.25 / q[3] * ( dcm[6] - dcm[2] );
            q[2] = 0.25 / q[3] * ( dcm[1] - dcm[3] );
        }

        return q;

    }


    this.dcm = function( qua ){
        //console.log('dcm');

        q1  = qua[0] * qua[0];
        q2  = qua[1] * qua[1];
        q3  = qua[2] * qua[2];
        q4  = qua[3] * qua[3];

        q12 = qua[0] * qua[1] * 2.0;
        q34 = qua[2] * qua[3] * 2.0;
        q31 = qua[2] * qua[0] * 2.0;
        q24 = qua[1] * qua[3] * 2.0;
        q23 = qua[1] * qua[2] * 2.0;
        q14 = qua[0] * qua[3] * 2.0;

        return [ q1 - q2 - q3 + q4, q12 + q34        , q31 - q24        ,
                 q12 - q34        , q2 - q3 - q1 + q4, q23 + q14        ,
                 q31 + q24        , q23 - q14        , q3 - q1 - q2 + q4 ];
    }

    this.dcmt = function( qua ){
        //console.log('dcmt');

        var dcm = this.dcm( qua );

        return [ dcm[0], dcm[3], dcm[6],
                 dcm[1], dcm[4], dcm[7],
                 dcm[2], dcm[5], dcm[8] ];
    }

    this.euler = function( dcm, flag = false ){
        //console.log('euler');
        //var D = 180.0 / Math.PI;
        var phi = Math.atan2( dcm[5], dcm[8]);
        var the = Math.atan2(-dcm[2], Math.sqrt( [5] * dcm[5] + dcm[8] * dcm[8] ) );
        var psi = Math.atan2( dcm[1], dcm[0]);
        if( flag ) { 
            the = - the;
            psi = - psi;
        }
        return [ phi,the, psi ];
    }


    this.getData = function( t, d ){
        var ts = ( t - d.t0 ) / d.dt;
        var k = Math.min( d.data.length - 2, Math.max( 0, Math.floor( ts ) ) );
        if( d.type == 'linear'){
            var p = ts - k;
            return sFn.add( sFn.mult( 1 - p, d.data[k] ), sFn.mult( p, d.data[k+1] ) ); 
        }else if( d.type == 'step'){
            return d.data[k]; 
        }
    }
}




//--------------------------------------------------------
//
//	êîäwä÷êî
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



