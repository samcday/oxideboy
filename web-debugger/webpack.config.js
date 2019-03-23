const path = require('path');
const LoaderOptionsPlugin = require('webpack').LoaderOptionsPlugin;
const HtmlWebpackPlugin = require('html-webpack-plugin');
const WasmPackPlugin = require('@wasm-tool/wasm-pack-plugin');

const dist = path.resolve(__dirname, 'dist');

module.exports = {
  devtool: 'eval-source-map',

  entry: './js/index',
  output: {
    globalObject: 'self',
    path: dist,
    filename: 'bundle.js',
  },
  devServer: {
    contentBase: dist,
  },
  mode: 'development',
  plugins: [
    new HtmlWebpackPlugin({template: 'index.html'}),
    new WasmPackPlugin({crateDirectory: path.resolve(__dirname, 'crate')}),
    new LoaderOptionsPlugin({
      options: {
        worker: {
          output: {
            filename: "hash.worker.js",
            chunkFilename: "[id].hash.worker.js"
          }
        }
      }
    }),
  ],
  resolve: {
    extensions: ['.ts', '.tsx', '.js', '.json', '.wasm']
  },
  module:{
    rules:[
      {
        
        test: /\.(js|ts)x?$/,
        exclude: /node_modules/,
        loader: 'babel-loader'
      },
      {
        test: /\.woff(2)?(\?v=[0-9]\.[0-9]\.[0-9])?$/, 
        loader: 'url-loader?limit=10000&mimetype=application/font-woff' 
      },
      { 
        test: /\.(ttf|eot|svg)(\?v=[0-9]\.[0-9]\.[0-9])?$/, 
        loader: 'file-loader' 
      },
      {
        test:/\.css$/,
        use:['style-loader','css-loader']
      },
      {
        test: /\.(scss)$/,
        use: [{
          loader: 'style-loader', // inject CSS to page
        }, {
          loader: 'css-loader', // translates CSS into CommonJS modules
        }, {
          loader: 'postcss-loader', // Run postcss actions
          options: {
            plugins: function () { // postcss plugins, can be exported to postcss.config.js
              return [
                require('autoprefixer')
              ];
            }
          }
        }, {
          loader: 'sass-loader' // compiles Sass to CSS
        }]
      },
   ]
  },
};
