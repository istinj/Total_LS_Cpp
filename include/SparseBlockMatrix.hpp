


namespace optimizer {
	template<class DenseMatrixType>
	SparseBlockMatrix<DenseMatrixType>::SparseBlockMatrix(const std::vector<int>& r_block_indices,
			const std::vector<int>& c_block_indices,
			const int n_rows_, const int n_cols_){
		_rowBlockIndices(r_block_indices);
		_colBlockIndices(c_block_indices);
		_blockRowsContainer.resize(r_block_indices.size());
	}
	template<class DenseMatrixType>
	SparseBlockMatrix<DenseMatrixType>::~SparseBlockMatrix(){
	}

}




